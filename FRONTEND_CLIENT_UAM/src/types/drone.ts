// models.ts - TypeScript types for drone data

// Battery information
interface BatteryInfo {
  remaining?: number;
  voltage?: number;
  temperature?: number;
}

// 3D position (North, East, Down coordinates)
interface Position {
  x: number;
  y: number;
  z: number;
}

// Orientation (roll, pitch, yaw)
interface Attitude {
  roll: number;
  pitch: number;
  yaw: number;
}

// GPS coordinates
interface GPSInfo {
  latitude: number;
  longitude: number;
  altitude: number;
}

// Velocity in NED frame
interface Velocity {
  north: number;
  east: number;
  down: number;
}

// Drone status information
interface DroneStatus {
  connected: boolean;
  armed: boolean;
  in_air: boolean;
  flight_mode: string;
}

// Complete drone information
interface DroneInfo {
  id: string;
  name: string;
  status: DroneStatus;
  battery?: BatteryInfo;
  position?: Position;
  attitude?: Attitude;
  gps?: GPSInfo;
  velocity?: Velocity;
}

// Command to send to a drone
interface DroneCommand {
  command: string;
  parameters?: Record<string, any>;
}

// Response from a drone API call
interface DroneResponse {
  success: boolean;
  message: string;
  data?: Record<string, any>;
}

// Message payload (for your message system)
interface MsgPayload {
  msg_id?: number;
  msg_name: string;
}

// Export all interfaces
export type {
  BatteryInfo,
  Position,
  Attitude,
  GPSInfo,
  Velocity,
  DroneStatus,
  DroneInfo,
  DroneCommand,
  DroneResponse,
  MsgPayload
};