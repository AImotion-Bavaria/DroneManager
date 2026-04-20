export interface MapType {
  id: number;
  name: string;
  image: string;
  active: boolean;
}

export interface DroneMarkerData {
  id: string;
  type: string;
  className: string;
  movedClassName: string;
  pulse: boolean;
  wsData: {
    position: number[];
    heading: number;
    conn: boolean;
    armed: boolean;
    in_air: boolean;
  };
}