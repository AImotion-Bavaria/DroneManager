export interface DroneStatus {
  connection: { text: string; color: string };
  armed: { text: string; color: string };
  flight: { text: string; color: string };
  time: string;
  speed: string;
}

export interface DroneData {
  id: number | string;
  name: string;
  image: string | null;
  status: DroneStatus;
}

// export interface DronePosition {
//   id: number;
//   className: string;
//   type: "standard" | "active";
//   pulse: boolean; // Indicates if the drone should pulse
//   movedClassName?: string; // Optional property for moved position
// }
// In types.ts
export interface DronePosition {
  id: string | number;
  name?: string;
  className: string;
  type: string;
  movedClassName?: string;
  pulse: boolean;
  wsData?: {
    position: [number, number, number];
    heading: number;
    conn: boolean;
    armed: boolean;
    in_air: boolean;
  };
}


