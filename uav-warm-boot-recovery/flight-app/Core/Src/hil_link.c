#include "hil_link.h"

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"
#include "usart.h"

#include <stdbool.h>
#include <string.h>

#define RX_BUFFER_SIZE 512U
#define RX_PROGRESS    (1U << 0)
#define TX_COMPLETE    (1U << 1)
#define UART_ERROR     (1U << 2)

static uint8_t rx_buffer[RX_BUFFER_SIZE];
static uint8_t tx_buffer[RX_BUFFER_SIZE];

static osThreadId_t volatile loopback_task;
static volatile uint32_t rx_produced;

/* The UART and DMA interrupts share a preemption priority.
 * Task-side sampling holds a critical section to serialize with them.
 */
static uint32_t sample_rx_progress(void)
{
  uint32_t produced = rx_produced;
  uint32_t position = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
  uint32_t previous = produced % RX_BUFFER_SIZE;

  /* Read the live DMA cursor so a delayed HT/TC event cannot move it backward.
   * At least one sample per ring lap is required. Keep both HT and TC enabled.
   */
  produced += (position + RX_BUFFER_SIZE - previous) % RX_BUFFER_SIZE;
  rx_produced = produced;
  return produced;
}

static uint32_t rx_progress_from_task(void)
{
  taskENTER_CRITICAL();
  uint32_t produced = sample_rx_progress();
  taskEXIT_CRITICAL();
  return produced;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
  (void)size;

  if (uart == &huart1 && loopback_task != NULL)
  {
    sample_rx_progress();
    osThreadFlagsSet(loopback_task, RX_PROGRESS);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
  if (uart == &huart1 && loopback_task != NULL)
  {
    osThreadFlagsSet(loopback_task, TX_COMPLETE);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
  if (uart == &huart1 && loopback_task != NULL)
  {
    osThreadFlagsSet(loopback_task, UART_ERROR);
  }
}

static const char *run_loopback(void)
{
  uint32_t consumed = 0U;
  bool tx_busy = false;

  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, sizeof(rx_buffer)) != HAL_OK)
  {
    return "RX DMA start failed";
  }

  for (;;)
  {
    uint32_t flags = osThreadFlagsWait(RX_PROGRESS | TX_COMPLETE | UART_ERROR,
                                       osFlagsWaitAny, osWaitForever);
    if ((flags & osFlagsError) != 0U)
    {
      return "task notification failed";
    }
    if ((flags & UART_ERROR) != 0U)
    {
      return "UART or DMA error";
    }
    if ((flags & TX_COMPLETE) != 0U)
    {
      tx_busy = false;
    }

    uint32_t available = rx_progress_from_task() - consumed;
    if (available > RX_BUFFER_SIZE)
    {
      return "RX buffer overrun";
    }
    if (available == 0U || tx_busy)
    {
      continue;
    }

    const volatile uint8_t *dma_data = rx_buffer;
    for (uint32_t index = 0U; index < available; ++index)
    {
      tx_buffer[index] = dma_data[(consumed + index) % RX_BUFFER_SIZE];
    }
    if (rx_progress_from_task() - consumed > RX_BUFFER_SIZE)
    {
      return "RX buffer overrun during copy";
    }

    if (HAL_UART_Transmit_DMA(&huart1, tx_buffer, (uint16_t)available) != HAL_OK)
    {
      return "TX DMA start failed";
    }
    consumed += available;
    tx_busy = true;
  }
}

void HilLink_LoopbackTask(void *argument)
{
  (void)argument;

  loopback_task = osThreadGetId();
  const char *error = run_loopback();

  /* Stop after the first failure so a corrupted run cannot appear to pass. */
  loopback_task = NULL;
  (void)HAL_UART_Abort(&huart1);

  static const uint8_t prefix[] = "USART1 loopback: ";
  static const uint8_t newline[] = "\r\n";
  (void)HAL_UART_Transmit(&huart2, prefix, sizeof(prefix) - 1U, 100U);
  (void)HAL_UART_Transmit(&huart2, (const uint8_t *)error, (uint16_t)strlen(error), 100U);
  (void)HAL_UART_Transmit(&huart2, newline, sizeof(newline) - 1U, 100U);
  osThreadExit();
}
