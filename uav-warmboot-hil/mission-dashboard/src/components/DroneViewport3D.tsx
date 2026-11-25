import { useRef } from "react";
import { Canvas, useFrame } from "@react-three/fiber";
import { ContactShadows, Grid, OrbitControls } from "@react-three/drei";
import type { Group } from "three";
import type { Telemetry } from "../types/telemetry";

const START_ALT = 30; // keep in sync with config.START_ALT; the view is drawn relative to it
const ARM = 0.42; // visual motor offset along each of X and Z
// Body FRD motor order FR, RL, FL, RR as (forward, right) signs; spin +1 = CCW (matches config.MOTOR_SPIN).
const MOTORS: { fwd: number; right: number; spin: number }[] = [
  { fwd: 1, right: 1, spin: 1 }, { fwd: -1, right: -1, spin: 1 },
  { fwd: 1, right: -1, spin: -1 }, { fwd: -1, right: 1, spin: -1 },
];

/** Spinning rotor: speed follows motor throttle, blur disc fades in as it speeds up. */
function Rotor({ throttle, spin }: { throttle: number; spin: number }) {
  const blades = useRef<Group>(null);
  useFrame((_, dt) => {
    if (blades.current) blades.current.rotation.y += spin * (4 + throttle * 60) * dt;
  });
  return (
    <group position={[0, 0.09, 0]}>
      <group ref={blades}>
        {[0, Math.PI / 2].map((a) => (
          <mesh key={a} rotation-y={a}>
            <boxGeometry args={[0.46, 0.008, 0.035]} />
            <meshStandardMaterial color="#1b1f26" />
          </mesh>
        ))}
      </group>
      <mesh>
        <cylinderGeometry args={[0.23, 0.23, 0.004, 32]} />
        <meshBasicMaterial color="#9fb3c8" transparent opacity={0.05 + 0.3 * throttle} depthWrite={false} />
      </mesh>
    </group>
  );
}

function Drone({ t }: { t: Telemetry }) {
  const [w, x, y, z] = t.quat;
  // NED -> three.js (X=east, Y=up, Z=-north). A proper rotation, so quaternion vector parts map
  // the same way: (x,y,z)_ned -> (y, -z, -x). Nose (body +x) therefore points to -Z.
  const quat: [number, number, number, number] = [y, -z, -x, w];
  const pos: [number, number, number] = [t.pos[1], -t.pos[2] - START_ALT, -t.pos[0]];
  const out = !t.mcu.online;
  const frame = out ? "#e5484d" : "#2b3440";
  const accent = out ? "#ff6b6b" : "#3fb6d9";

  return (
    <group position={pos} quaternion={quat}>
      {/* X-frame arms */}
      {[Math.PI / 4, -Math.PI / 4].map((a) => (
        <mesh key={a} rotation-y={a}>
          <boxGeometry args={[ARM * 2.83, 0.035, 0.07]} />
          <meshStandardMaterial color={frame} metalness={0.5} roughness={0.4} emissive={out ? "#7a1c20" : "#000"} />
        </mesh>
      ))}
      {/* Fuselage + canopy + battery */}
      <mesh>
        <boxGeometry args={[0.26, 0.07, 0.34]} />
        <meshStandardMaterial color={frame} metalness={0.6} roughness={0.35} emissive={out ? "#7a1c20" : "#000"} />
      </mesh>
      <mesh position={[0, 0.06, -0.04]} scale={[1, 0.55, 1.35]}>
        <sphereGeometry args={[0.1, 24, 16]} />
        <meshStandardMaterial color={accent} metalness={0.3} roughness={0.2} />
      </mesh>
      <mesh position={[0, -0.06, 0.02]}>
        <boxGeometry args={[0.16, 0.05, 0.26]} />
        <meshStandardMaterial color="#12161c" />
      </mesh>
      {/* Heading LEDs: green nose, red tail */}
      <mesh position={[0, 0.02, -0.18]}>
        <sphereGeometry args={[0.025, 12, 12]} />
        <meshBasicMaterial color="#30e08a" />
      </mesh>
      <mesh position={[0, 0.02, 0.18]}>
        <sphereGeometry args={[0.025, 12, 12]} />
        <meshBasicMaterial color="#ff4d4d" />
      </mesh>
      {/* Motors, rotors, landing legs */}
      {MOTORS.map((m, i) => (
        <group key={i} position={[m.right * ARM, 0, -m.fwd * ARM]}>
          <mesh position={[0, 0.04, 0]}>
            <cylinderGeometry args={[0.05, 0.055, 0.08, 20]} />
            <meshStandardMaterial color="#151a21" metalness={0.8} roughness={0.3} />
          </mesh>
          <mesh position={[0, 0.085, 0]}>
            <cylinderGeometry args={[0.02, 0.02, 0.02, 12]} />
            <meshStandardMaterial color={accent} />
          </mesh>
          <Rotor throttle={t.motors[i]} spin={m.spin} />
          <mesh position={[0, -0.09, 0]}>
            <cylinderGeometry args={[0.012, 0.012, 0.16, 8]} />
            <meshStandardMaterial color="#12161c" />
          </mesh>
        </group>
      ))}
    </group>
  );
}

/** X-frame quad over a grid fixed at the hover reference height. Turns red while the MCU is silent. */
export function DroneViewport3D({ t }: { t: Telemetry | null }) {
  return (
    <Canvas camera={{ position: [1.8, 1.1, 2.2], fov: 45 }} shadows>
      <color attach="background" args={["#0b0f14"]} />
      <fog attach="fog" args={["#0b0f14", 8, 30]} />
      <ambientLight intensity={0.5} />
      <directionalLight position={[3, 5, 2]} intensity={1.6} />
      <pointLight position={[-3, 1, -2]} intensity={0.6} color="#6fa8ff" />
      <Grid position={[0, -0.6, 0]} args={[40, 40]} cellColor="#2a3441" sectionColor="#3d4b5c" fadeDistance={22} infiniteGrid />
      <ContactShadows position={[0, -0.59, 0]} opacity={0.5} scale={4} blur={2.5} far={2} />
      {t && <Drone t={t} />}
      <OrbitControls makeDefault target={[0, 0, 0]} maxPolarAngle={Math.PI / 2 - 0.05} />
    </Canvas>
  );
}
