// @ts-nocheck
import { useState, useEffect, useRef } from "react";
import { DronePosition } from "./types";
import { cn } from "../../lib/utils";
import { useConnectionStore } from "../../stores/connection-store";

const DURATION = 500; // Reduced for smoother movement

interface DroneMarkerProps {
  drone: DronePosition;
  wsData: {
    position: [number, number, number];
    attitude?: [number, number, number];
    heading: number;
    conn: boolean;
    armed: boolean;
    in_air: boolean;
  };
}

export const DroneMarker = ({
  drone,
  wsData,
}: DroneMarkerProps): JSX.Element => {
  const [isAnimating, setIsAnimating] = useState(false);
  const [isPulsing, setIsPulsing] = useState(wsData?.in_air || false);
  const [position, setPosition] = useState({ left: "45%", top: "25%" });
  const [continuousHeading, setContinuousHeading] = useState(0);
  const previousHeadingRef = useRef<number | null>(null);

  useEffect(() => {
    if (wsData?.position) {
      const [x, y] = wsData.position;

      // Convert coordinates to percentage positions
      // const calculatePosition = (x: number, y: number) => {
      //   // Physical space constraints
      //   const SPACE_WIDTH = 6; // 6 meters
      //   const SPACE_HEIGHT = 2; // 2 meters
      //   const MAX_MOVEMENT = 3; // 3 meters max movement
      //   const ZOOM_FACTOR = 0.7; // Zoom in to 70% of the space for better visibility

      //   // Clamp the position within the movement range
      //   const clampedX = Math.max(-MAX_MOVEMENT, Math.min(MAX_MOVEMENT, x));
      //   const clampedY = Math.max(-MAX_MOVEMENT, Math.min(MAX_MOVEMENT, y));

      //   // Map the physical coordinates to percentage with zoom
      //   // Center point is 50%, then add the relative position scaled by zoom
      //   const leftPercent = 50 + ((clampedX / (SPACE_WIDTH/2)) * 50 * ZOOM_FACTOR);
      //   const topPercent = 50 + ((-clampedY / (SPACE_HEIGHT/2)) * 50 * ZOOM_FACTOR);

      //   // Add minimum spacing between drones (8% of container for better visibility)
      //   const DRONE_SPACING = 8;
        
      //   return {
      //     left: `${Math.max(DRONE_SPACING, Math.min(100 - DRONE_SPACING, leftPercent))}%`,
      //     top: `${Math.max(DRONE_SPACING, Math.min(100 - DRONE_SPACING, topPercent))}%`
      //   };
      // };



      // NEW ONE I CHANGED
       // Convert coordinates to percentage positions
       const calculatePosition = (x: number, y: number) => {
        // Physical space constraints
        const SPACE_WIDTH = 6; // 6 meters
        const SPACE_HEIGHT = 2; // 2 meters
        const MAX_MOVEMENT = 3; // 3 meters max movement
        const ZOOM_FACTOR = 0.7; // Zoom in to 70% of the space for better visibility

        // Clamp the position within the movement range
        const clampedX = Math.max(-MAX_MOVEMENT, Math.min(MAX_MOVEMENT, x));
        const clampedY = Math.max(-MAX_MOVEMENT, Math.min(MAX_MOVEMENT, y));

        // Invert the X coordinate by multiplying by -1
        const invertedX = -clampedX;

        // Map the physical coordinates to percentage with zoom
        // Center point is 50%, then add the relative position scaled by zoom
        const leftPercent = 50 + ((invertedX / (SPACE_WIDTH/2)) * 50 * ZOOM_FACTOR);
        const topPercent = 50 + ((-clampedY / (SPACE_HEIGHT/2)) * 50 * ZOOM_FACTOR);

        // Add minimum spacing between drones (8% of container for better visibility)
        const DRONE_SPACING = 8;
        
        return {
          left: `${Math.max(DRONE_SPACING, Math.min(100 - DRONE_SPACING, leftPercent))}%`,
          top: `${Math.max(DRONE_SPACING, Math.min(100 - DRONE_SPACING, topPercent))}%`
        };
      };

      const newPosition = calculatePosition(x, y);
      setPosition(newPosition);
      setIsPulsing(wsData.in_air);
    }
  }, [wsData]);

  useEffect(() => {
    const yawFromAttitude =
      Array.isArray(wsData?.attitude) && wsData.attitude.length >= 3
        ? wsData.attitude[2]
        : undefined;
    const angleSource =
      typeof yawFromAttitude === "number" && Number.isFinite(yawFromAttitude)
        ? yawFromAttitude
        : wsData?.heading;

    if (typeof angleSource !== "number") return;

    const normalizedHeading = ((angleSource % 360) + 360) % 360;

    if (previousHeadingRef.current === null) {
      previousHeadingRef.current = normalizedHeading;
      setContinuousHeading(normalizedHeading);
      return;
    }

    // Unwrap heading changes so 359 -> 0 rotates forward by 1 degree.
    let delta = normalizedHeading - previousHeadingRef.current;
    if (delta > 180) delta -= 360;
    if (delta < -180) delta += 360;

    setContinuousHeading((prev) => prev + delta);
    previousHeadingRef.current = normalizedHeading;
  }, [wsData?.attitude, wsData?.heading]);


  // console.log(wsData)
  return (
    <div
      key={drone.id}
      className={cn(
        "absolute transform -translate-x-1/2 -translate-y-1/2 transition-all duration-500",
        "hover:z-50" // Ensure hovered drone appears above others
      )}
      style={{
        left: position.left,
        top: position.top,
        zIndex: wsData?.in_air ? 40 : 30 // Flying drones appear above grounded ones
      }}
    >
      {/* Drone name label */}
      <div className="absolute -top-6 left-0 w-full text-center text-xs font-semibold text-white   py-0.5 rounded-full">
        {drone.id.charAt(0).toUpperCase() + drone.id.slice(1)}
      </div>

      <div className="relative w-14 h-[55px]">
        {/* Shadow/background that rotates with the drone */}
        <div
          className="absolute w-full h-full origin-center"
          style={{
            transform: `rotate(${continuousHeading + 180}deg)`,
            transition: "transform 500ms ease-out",
          }}
        >
          <img
            className="absolute w-[60px] h-[55px] top-1/2 left-1/2 -translate-x-1/4 -translate-y-1/2"
            alt="Polygon"
            src="./polygon-3.svg"
          />

<img
            className="absolute w-[10px] ml-6 h-[10px] top-1/2 left-1/2 -translate-x-1/4 -translate-y-1/2"
            alt="Polygon"
            src="./polygon-1.svg"
          />

        </div>

        {/* Drone body */}
        <div className="absolute w-[55px] h-[38px] top-2.5 left-1/2 -translate-x-1/2">
          <div
            className={cn(
              "absolute w-[38px] h-[38px] top-0 left-1/2 -translate-x-1/2",
              wsData?.conn ? "bg-[#54cde2]" : "bg-[#d9d9d9]",
              "rounded-[19px] overflow-hidden"
              // wsData?.in_air && "border-2 border-orange-600 animate-[pulse_2s_ease-in-out]"
            )}
          >
            <img
              className="absolute w-5 h-5 top-[9px] left-[9px] transition-transform duration-500"
              alt="Quadcopter"
              src="./quadcopter.svg"
              style={{
                transform: `rotate(${continuousHeading}deg)`,
              }}
            />
          </div>
        </div>
      </div>

      {/* Status indicators */}
      {/* <div className="absolute -bottom-2 left-0 w-full flex justify-center gap-1">
        <div
          className={cn(
            "w-2 h-2 rounded-full",
            wsData?.conn ? "bg-green-500" : "bg-red-500"
          )}
        />
        <div
          className={cn(
            "w-2 h-2 rounded-full",
            wsData?.in_air ? "bg-blue-500" : "bg-gray-500"
          )}
        />
      </div> */}
    </div>
  );
};
