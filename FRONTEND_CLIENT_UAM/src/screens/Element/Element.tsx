import { useState, useEffect } from "react";
import { Header } from "../../components/layout/Header";
import { Header1 } from "../../components/layout/SubHeader";
import { MapView } from "../../components/map/MapView";
import { MapTypeSelector } from "../../components/map/MapTypeSelector";
import { MapControls } from "../../components/map/MapControls";
import { mapTypes } from "../../data/mapData";
import { MapType } from "../../components/map/types";
import MapCircle from "../../components/map/MapCircle";
import DroneCardsSidebar from "../../components/drones/DroneCardsSidebar";
import { AlertTriangle, Battery, RotateCw } from "lucide-react";
import { useConnectionStore } from "../../stores/connection-store";
import { useFlyDrone } from "../../api/useDrone";

export const Element = (): JSX.Element => {
  const [activeMapTypes, setActiveMapTypes] = useState<MapType[]>(mapTypes);
  const [showBatteryLowWarning, setShowBatteryLowWarning] = useState(false);
  const {
    connected,
    setSwapped,
    searchStarted,
    setDronePoss,
    dronePoss: dronePositions,
  } = useConnectionStore();
  const { mutate: swapMutation } = useFlyDrone();

  // Simulate battery low warning after 15 seconds
  useEffect(() => {
    if (connected && searchStarted) {
      const timer = setTimeout(() => {
        setShowBatteryLowWarning(true);
      }, 15000);

      return () => clearTimeout(timer);
    } else {
      setShowBatteryLowWarning(false);
    }
  }, [connected, searchStarted]);

  const handleMapTypeSelect = (id: number) => {
    setActiveMapTypes(
      activeMapTypes.map((mapType) => ({
        ...mapType,
        active: mapType.id === id,
      }))
    );
  };

  const handleZoomIn = () => {
    console.log("Zoom in");
    // Implement zoom in functionality
  };

  const handleZoomOut = () => {
    console.log("Zoom out");
    // Implement zoom out functionality
  };

  const initialCnames = {
    1: "top-[25%] left-[45%] md:top-[25%] md:left-[20%]",
  };
  const handleSwap = async () => {
    try {
      setShowBatteryLowWarning(false);

      console.log("Moving standby drone to active drone position...");
      setDronePoss(
        dronePositions.map((drone) => {
          if (drone.id === 2) {
            return {
              ...drone,
              className: drone.movedClassName || "",
              pulse: true,
            };
          }
          return drone;
        })
      );

      await new Promise((resolve) => setTimeout(resolve, 2500));

      console.log("Moving active drone back to base...");
      setDronePoss(
        dronePositions.map((drone) => {
          if (drone.id === 1) {
            return {
              ...drone,
              className: initialCnames[1],
              pulse: false,
            };
          } else if (drone.id === 2) {
            return {
              ...drone,
              className: drone.movedClassName || "",
              pulse: true,
            };
          }
          return drone;
        })
      );

      await new Promise((resolve) => setTimeout(resolve, 1500));

      console.log("Turning off initial drone...");
      setDronePoss(
        dronePositions.map((drone) => {
          if (drone.id === 1) {
            return {
              ...drone,
              type: "standard",
              className: "rotate-180" + initialCnames[1],
              pulse: false,
            };
          } else if (drone.id === 2) {
            return {
              ...drone,
              className: drone.movedClassName || "",
              pulse: true,
            };
          }
          return drone;
        })
      );

      // Update the global state to reflect the swap
      setSwapped(true);
    } catch (error) {
      console.error("Error during  swap:", error);
      setShowBatteryLowWarning(true);
    }
  };

  // use this function to swap the drones on the server after drone are connected
  const handleServerSwap = () => {
    swapMutation(
      {
        droneId: "2",
      },
      {
        onSuccess: async () => {
          setShowBatteryLowWarning(false);

          setDronePoss(
            dronePositions.map((drone) => {
              if (drone.id === 2) {
                return {
                  ...drone,
                  className: drone.movedClassName || "",
                  pulse: true,
                };
              }
              return drone;
            })
          );
        },
        onError: () => {
          alert("Error during swap");
          setShowBatteryLowWarning(true);
        },
      }
    );
    swapMutation(
      {
        droneId: "1",
        data: {
          x: 0,
          y: 0,
          z: 0,
          yaw: 0,
        },
      },
      {
        onSuccess: async () => {
          setDronePoss(
            dronePositions.map((drone) => {
              if (drone.id === 1) {
                return {
                  ...drone,
                  className: initialCnames[1],
                  pulse: false,
                };
              } else if (drone.id === 2) {
                return {
                  ...drone,
                  className: drone.movedClassName || "",
                  pulse: true,
                };
              }
              return drone;
            })
          );

          await new Promise((resolve) => setTimeout(resolve, 1500));

          console.log("Turning off initial drone...");
          setDronePoss(
            dronePositions.map((drone) => {
              if (drone.id === 1) {
                return {
                  ...drone,
                  type: "standard",
                  className: "rotate-180" + initialCnames[1],
                  pulse: false,
                };
              } else if (drone.id === 2) {
                return {
                  ...drone,
                  className: drone.movedClassName || "",
                  pulse: true,
                };
              }
              return drone;
            })
          );

          // Update the global state to reflect the swap
          setSwapped(true);
        },
        onError: () => {
          alert("Error during swap");
          // Re-enable the warning in case of error
          setShowBatteryLowWarning(true);
        },
      }
    );
  };
  return (
    <>
      <div className="bg-[#353535] flex relative">
        {/* Map Circle and Controls Container */}
        <div className="fixed left-0 bottom-0 z-50 p-4 flex flex-col items-start gap-4">
          {/* Map Circle */}
          <div className="relative">
            <MapCircle />
          </div>
          
          {/* Map Type Selector - Positioned relative to MapCircle */}
          <div className="absolute left-[calc(100%+1rem)] bottom-[35%] flex gap-4">
            {/* <MapTypeSelector
              mapTypes={activeMapTypes}
              onSelect={handleMapTypeSelect}
            /> */}
            {/* <MapControls onZoomIn={handleZoomIn} onZoomOut={handleZoomOut} /> */}
          </div>
        </div>

        {/* Battery Low Warning and Swap Button - center bottom */}
        <div className="fixed left-1/2 transform -translate-x-1/2 bottom-4 z-50">
          {showBatteryLowWarning && (
            <div className="flex items-center bg-black/70 backdrop-blur-sm rounded-lg overflow-hidden animate-bounce">
              <div className="bg-red-600 py-2 px-3 flex items-center">
                <AlertTriangle className="h-5 w-5 text-white mr-2" />
                <span className="font-medium text-white text-sm">
                  Battery Low
                </span>
              </div>
              <div className="p-2 px-4">
                <div className="flex items-center">
                  <Battery className="h-5 w-5 text-red-500 mr-2" />
                  <span className="text-white text-sm mr-4">25% Remaining</span>
                  <button
                    onClick={handleSwap}
                    className="bg-blue-600 hover:bg-blue-700 text-white py-1.5 px-3 rounded-md flex items-center text-sm transition-colors"
                  >
                    <RotateCw className="h-4 w-4 mr-2" />
                    Swap
                  </button>
                </div>
              </div>
            </div>
          )}
        </div>


        {/* Content container */}
        <div className="flex-1 overflow-y-auto">
          <Header />
          <Header1 />
          {/* Main content area */}
          <div className="flex gap-8 p-8">
            <div className="flex-1 items-start align-top">
              <MapView />
            </div>

            {/* Sidebar container - DroneCardsSidebar handles responsiveness */}
            <div className="w-[315px] flex-col justify-between">
              <DroneCardsSidebar />
            </div>
          </div>
        </div>
      </div>
    </>
  );
};
