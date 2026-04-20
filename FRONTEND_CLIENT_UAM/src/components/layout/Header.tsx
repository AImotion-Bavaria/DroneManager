import { useState, useEffect } from "react";
import { XIcon, WifiIcon } from "lucide-react";
import { useConnectionStore } from "../../stores/connection-store";
import { Wifi, WifiOff } from "lucide-react";
import {
  useConnectDrone,
  useDisconnectDrone,
  useGetDrones,
} from "../../api/useDrone";

export const Header = (): JSX.Element => {
  const [connectionError, setConnectionError] = useState<string | null>(null);
  const [showConnectionPopup, setShowConnectionPopup] = useState(false);
  const [droneId, setDroneId] = useState(
    import.meta.env.VITE_DEFAULT_DRONE_NAME || "drone1"
  );
  const [connectionString, setConnectionString] = useState(
    import.meta.env.VITE_DEFAULT_UDP || "udp://:14540"
  );

  const { connected: isConnected, setConnected: setIsConnected, dmConnected, setDmConnected } =
    useConnectionStore();

  // Wire up Electron IPC dm-status (UDP probe result from main process)
  useEffect(() => {
    const api = (window as any).electronAPI;
    if (!api?.onDMStatus) return;
    api.onDMStatus((status: boolean) => {
      setDmConnected(status);
    });
    return () => {
      api.removeDMStatusListener?.();
    };
  }, [setDmConnected]);

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

  const handleDisconnect = async () => {
    disconnectDrone(droneId);
    setIsConnected(false);
  };

  return (
    <div className="h-[57px] flex items-center bg-[#414141] px-6 relative">
      <img
        src="./Logo_TTZ_w.svg"
        alt="TTZ Logo"
        className="h-[40px] w-auto flex-shrink-0 object-contain"
      />

      {/* <div className="ml-6 flex ">

        <div className=" w-[100px] border-l h-full  border-gray-500 pl-4 bg-[#353535]">
          <span className="text-[#dbdbdb] text-xs font-medium">SAR Operation</span>
          <button className="hover:bg-[#3d3d3d] p-1 rounded">
            <XIcon className="w-3 h-3 text-white" />
          </button>
        </div>
      </div> */}
      <div className="ml-6 flex ">
        <div className="flex items-center bg-[#353535] px-4 py-3 rounded-[8px]">
          <span className="text-[#dbdbdb] text-xs font-medium mr-2">SAR Operation</span>
          <button className="hover:bg-[#3d3d3d] p-1 rounded">
            <XIcon className="w-3 h-3 text-white" />
          </button>
        </div>
      </div>

      {/* DM Connection Status */}
      <div className="ml-auto flex items-center">
        <div className={`flex items-center gap-2 px-3 py-1.5 rounded-[6px] ${dmConnected ? 'bg-[#1a3a1a]' : 'bg-[#3a1a1a]'}`}>
          {dmConnected ? (
            <Wifi className="w-3.5 h-3.5 text-green-400" />
          ) : (
            <WifiOff className="w-3.5 h-3.5 text-red-400" />
          )}
          <span className={`text-xs font-medium ${dmConnected ? 'text-green-400' : 'text-red-400'}`}>
            DM {dmConnected ? 'Connected' : 'Offline'}
          </span>
          <div className={`w-1.5 h-1.5 rounded-full ${dmConnected ? 'bg-green-400 animate-pulse' : 'bg-red-400'}`} />
        </div>
      </div>

      {/* <div className="ml-auto flex items-center gap-4">
        {isConnected ? (
          <>
            <div className="flex items-center gap-2">
              <div className="w-2 h-2 bg-green-400 rounded-full animate-pulse"></div>
              <span className="text-white text-xs">Connected to {droneId}</span>
            </div>
            <button
              onClick={handleDisconnect}
              className="bg-red-500 hover:bg-red-600 text-white px-3 py-1 rounded text-xs transition-colors"
            >
              Disconnect
            </button>
          </>
        ) : (
          <button
            onClick={() => setShowConnectionPopup(true)}
            className="bg-blue-500 hover:bg-blue-600 text-white flex items-center gap-1 px-3 py-1 rounded text-xs transition-colors"
          >
            <WifiIcon className="w-3 h-3" />
            Connect Drone
          </button>
        )}
      </div> */}

      {/* Connection Popup */}
      {showConnectionPopup && (
        <div className="absolute top-full right-4 mt-2 w-72 bg-[#2d2d2d] rounded-md shadow-lg border border-gray-700 z-10">
          <div className="p-4">
            <div className="flex justify-between items-center mb-4">
              <h3 className="text-white text-sm font-medium">
                Connect to Drone
              </h3>
              <button
                onClick={() => setShowConnectionPopup(false)}
                className="text-gray-400 hover:text-white"
              >
                <XIcon className="w-4 h-4" />
              </button>
            </div>

            <div className="space-y-3">
              <div>
                <label className="block text-gray-300 text-xs mb-1">
                  Drone ID
                </label>
                <input
                  type="text"
                  value={droneId}
                  onChange={(e) => setDroneId(e.target.value)}
                  className="w-full bg-[#3d3d3d] border border-gray-700 rounded px-3 py-2 text-sm text-white"
                  placeholder="drone1"
                />
              </div>

              <div>
                <label className="block text-gray-300 text-xs mb-1">
                  Connection String
                </label>
                <input
                  type="text"
                  value={connectionString}
                  onChange={(e) => setConnectionString(e.target.value)}
                  className="w-full bg-[#3d3d3d] border border-gray-700 rounded px-3 py-2 text-sm text-white"
                  placeholder="udp://:14540"
                />
              </div>

              {connectionError && (
                <div className="text-red-400 text-xs py-1">
                  Error: {connectionError}
                </div>
              )}

              <button
                onClick={handleConnect}
                disabled={isConnecting}
                className={`w-full rounded py-2 text-white text-sm font-medium
                  ${isConnecting
                    ? "bg-blue-700 cursor-wait"
                    : "bg-blue-600 hover:bg-blue-700"
                  }`}
              >
                {isConnecting ? "Connecting..." : "Connect"}
              </button>
            </div>
          </div>

          <div className="border-t border-gray-700 p-3">
            <div className="flex items-center text-xs text-gray-400">
              <WifiIcon className="w-3 h-3 mr-1" />
              <span>For PX4 SITL use: udp://:14540</span>
            </div>
          </div>
        </div>
      )}


    </div>
  );
};
