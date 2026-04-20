// @ts-nocheck
import { useState, useEffect } from "react";
import { useConnectionStore } from "../../stores/connection-store";
import { Button } from "../../components/ui/button";
import { useWebSocket } from "../../hooks/useWebSocket";
import {
  useConnectDrone,
  useDisconnectDrone,
  useGetDrones,
} from "../../api/useDrone";

export const Header1 = (): JSX.Element => {
  const { droneData } = useWebSocket();
  const [connectionError, setConnectionError] = useState<string | null>(null);
  const [showConnectionPopup, setShowConnectionPopup] = useState(false);
  const [droneId, setDroneId] = useState(
    import.meta.env.VITE_DEFAULT_DRONE_NAME || "drone1"
  );
  const [connectionString, setConnectionString] = useState(
    import.meta.env.VITE_DEFAULT_UDP || "udp://:14540"
  );

  const { connected: isConnected, setConnected: setIsConnected } =
    useConnectionStore();

  // Use React Query hooks
  const {
    mutate: connectDrone,
    isPending: isConnecting,
    error: connectError,
  } = useConnectDrone();

  const { mutate: disconnectDrone } = useDisconnectDrone();

  const { data: dronesData, isSuccess: isDronesLoaded } = useGetDrones();

  // Check for existing drone connections when drones data is loaded
  useEffect(() => {
    if (isDronesLoaded && dronesData && dronesData.length > 0) {
      setIsConnected(true);
      setDroneId(dronesData[0].id);
    }
  }, [isDronesLoaded, dronesData, setIsConnected]);

  // Handle connection errors
  useEffect(() => {
    if (connectError) {
      setConnectionError(
        connectError instanceof Error
          ? connectError.message
          : "Connection failed"
      );
    }
  }, [connectError]);

  const handleConnect = async () => {
    setConnectionError(null);
    connectDrone(
      {
        drone_id: droneId,
        connection_string: connectionString,
      },
      {
        onSuccess: () => {
          setIsConnected(true);
          setShowConnectionPopup(false);
        },
        onError: (error) => {
          setConnectionError(
            error instanceof Error ? error.message : "Connection failed"
          );
          setIsConnected(true);
        },
      }
    );
  };


  // Check for low battery drones
  const getLowBatteryDrones = () => {
    if (!droneData?.missions) return [];

    const lowBattSet = new Set<string>();
    Object.entries(droneData.missions).forEach(([_, mission]) => {
      if (mission?.bat) {
        Object.entries(mission.bat).forEach(([droneName, batteryLevel]) => {
          // Check if battery is low AND drone is currently in the air
          const isFlying = droneData.drones[droneName]?.in_air || 
                          droneData.drones[droneName.toLowerCase()]?.in_air;
          if (typeof batteryLevel === 'number' && batteryLevel < 0.33 && isFlying) {
            lowBattSet.add(droneName);
          }
        });
      }
    });
    return Array.from(lowBattSet);
  };

  const lowBatteryDrones = getLowBatteryDrones();
  // Get total number of drones
  const getDroneCount = () => {
    if (!droneData?.drones) return 0;
    return Object.keys(droneData.drones).length;
  };

  const droneCount = getDroneCount();
  const droneCountMessage = `${droneCount} Drone${droneCount !== 1 ? 's' : ''} detected`;

  // Dynamically get the active mission stage (e.g., SearchGroup, POIFound, etc)
  const getMissionStage = () => {
    if (!droneData?.missions) return "Mission";
    const missions = Object.values(droneData.missions);
    if (missions.length > 0 && missions[0].stage) {
      return missions[0].stage;
    }
    return "Mission";
  };
  const activeMissionStage = getMissionStage();


  // Custom logic for SWAPING button and low battery warning

  const blueIsInAir = !!droneData?.drones?.blue?.in_air;
  const yellowIsInAir = !!droneData?.drones?.yellow?.in_air;
  const greenIsOnGround = droneData?.drones?.green ? !droneData.drones.green.in_air : false;
  const yellowBattery = droneData?.missions?.uam?.bat?.yellow;

  // Show SWAPING button only if blue and yellow are in air, green is on ground
  const showSwappingButton = blueIsInAir && yellowIsInAir && greenIsOnGround;

  // Show low battery warning only if yellow is in air, all others are on ground, and yellow battery < 0.33
  const drones = droneData?.drones || {};
  const droneNames = Object.keys(drones);
  const yellowIsOnlyInAir = yellowIsInAir && droneNames.every(name => {
    if (name === 'yellow') return true;
    return !drones[name]?.in_air;
  });
  const showLowBatteryWarning = yellowIsOnlyInAir && typeof yellowBattery === 'number' && yellowBattery < 0.50;
  const batteryWarningMessage = showLowBatteryWarning
    ? `Battery yellow is low!`
    : "";

  return (
    <>
      <div className="flex items-center justify-between w-full h-[65px] bg-[#2D2D2D] px-4 mb-3 rounded-[4px]">
        {/* Left section */}
        <div className="flex-1 flex items-center gap-4">
          <Button
            variant="outline"
            className="flex justify-start ml-4 items-center gap-2 h-[32px] w-[240px] px-3 bg-[#e8e8e85d] rounded-[8px] border-none backdrop-blur-[24px] hover:bg-[#3d3d3d] transition-colors"
          >
            <img className="w-4 h-4" alt="Mission" src="./vector-5.svg" />
            <span className="text-white text-xs font-medium">Mission</span>
            <img className="-rotate-[90deg] ml-1" alt="Vector" src="./Vector.png" />
            <span className="text-[#dbdbdb] text-xs font-medium truncate">{activeMissionStage}</span>
          </Button>
        </div>

        {/* Center section - Battery Warning / Stage info */}
        <div className="flex-1 flex justify-center items-center">
          {(showLowBatteryWarning || showSwappingButton) && (
            <div className="flex items-center gap-3 px-6 py-3 bg-[#3d3d3d] rounded-lg">
              {showLowBatteryWarning && (
                <div className="flex items-center gap-2">
                  <img className="w-5 h-5" alt="Warning" src="./error1.png" />
                  <span className="text-white text-sm">{batteryWarningMessage}</span>
                </div>
              )}
              {showSwappingButton && (
                <Button
                  variant="outline"
                  className="h-[32px] px-8 bg-transparent border-[#54cde2] text-[#54cde2] text-xs font-medium rounded-[10px] hover:bg-[#54cde2] hover:text-black"
                  onClick={() => console.log("Triggering Swap sequence...")}
                >
                  SWAPING..
                </Button>
              )}
            </div>
          )}
        </div>

        {/* Right section */}
        <div className="flex-1 flex items-center justify-end gap-4">
          <div className="flex items-center gap-2 mr-8">
            <div className={`w-6 h-6 ${droneCount > 0 ? 'bg-green-400' : 'bg-red-400'} rounded-full`}></div>
            <span className="text-white text-xs ml-2">{droneCountMessage}</span>
          </div>
          <Button
            variant="outline"
            className="h-[32px] px-4 bg-transparent border-white text-xs text-white font-medium rounded-[10px] hover:bg-black hover:text-white"
          >
            3 Drones Connected
          </Button>
        </div>
      </div>
    </>
  );
};
