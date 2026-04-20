// @ts-nocheck
import { XIcon } from "lucide-react";
import { Card, CardContent } from "../../components/ui/card";
import { DroneData } from "./types";
import { useEffect, useRef, useState } from "react";

interface DroneCardProps {
  drone: DroneData;
}

declare global {
  interface Window {
    JSMpeg: any;
  }
}

// const hostname = 'M4001-23-01.local';
const currentHostname =
  typeof window !== "undefined" && window.location.hostname
    ? window.location.hostname
    : "localhost";
const configuredLanHost = import.meta.env.VITE_LAN_HOST?.trim();
const hostname = currentHostname.startsWith("192.")
  ? currentHostname
  : configuredLanHost?.startsWith("192.")
    ? configuredLanHost
    : currentHostname;

const WS_PORT_BY_STREAM: Record<string, number> = {
  cameraXXX: 9991,
  vvvvv: 9992,
  ooo: 9993,
  derek: 9991,
  corran: 9992,
  luke: 9993,
  green: 9991,
  yellow: 9992,
  blue: 9993,
};

const getWsPortForDrone = (droneIdOrName: string): number => {
  return WS_PORT_BY_STREAM[droneIdOrName] ?? 9991;
};

export const DroneCard = ({ drone }: DroneCardProps): JSX.Element => {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const playerRef = useRef<any>(null);
  const [status, setStatus] = useState<string>('Initializing...');
  const [isConnected, setIsConnected] = useState<boolean>(false);

  useEffect(() => {
    const script = document.createElement('script');
    // script.src = '/src/components/drones/jsmpeg.min.js';
    script.src = '/jsmpeg.min.js'; 
    script.async = true;
    
    script.onload = () => {
      if (canvasRef.current) {
        startStream();
      }
    };
    
    document.body.appendChild(script);

    return () => {
      stopStream();
      document.body.removeChild(script);
    };
  }, []);

  const startStream = () => {
    if (playerRef.current) {
      playerRef.current.stop();
    }

    setStatus('Connecting to stream...');
    setIsConnected(false);

    try {
      // Match frontend card stream to backend RTSP websocket ports.
      const wsPort = getWsPortForDrone(drone.id);

                  //    const wsPort = 9991 + index;
                

      playerRef.current = new window.JSMpeg.Player(`ws://${hostname}:${wsPort}`, {
        canvas: canvasRef.current,
        audio: false,
        pauseWhenHidden: false,
        onSourceEstablished: () => {
          setStatus('Connected to stream');
          setIsConnected(true);
        },
        onSourceCompleted: () => {
          setStatus('Stream ended');
          setIsConnected(false);
        },
        onError: (error: any) => {
          setStatus(`Error: ${error}`);
          setIsConnected(false);
        }
      });
    } catch (error: any) {
      console.error('Error creating player:', error);
      setStatus(`Error: ${error.message}`);
      setIsConnected(false);
    }
  };

  const stopStream = () => {
    if (playerRef.current) {
      playerRef.current.stop();
      setStatus('Stream stopped');
      setIsConnected(false);
      playerRef.current = null;
    }
  };

  return (
    <Card className="bg-[#3535354c] rounded-[10px] border border-solid border-[#ffffff1a] backdrop-blur-[5p]">
      <div className="p-[18px] pb-0 flex justify-between items-center">
        <span className="font-normal text-white text-xs">
          {drone.name.charAt(0).toUpperCase() + drone.name.slice(1)}
        </span>
        <XIcon className="w-2.5 h-2.5 text-white cursor-pointer" />
      </div>

      <CardContent className="p-0 mt-4 rounded-[20px]  h-[206px] relative">
        <canvas 
          ref={canvasRef}
          className="w-full h-full object-content rounded-[10px]"
        />
        <div className="absolute bottom-0 left-0 right-0 h-[61px] bg-[#b2b1b14c] rounded-[10px] backdrop-blur-[3px] p-2">
          <div className="grid grid-cols-3 gap-2 text-xs text-white">
            {/* <div className="flex items-center gap-2">
              <div className={`w-2 h-2 ${isConnected ? 'bg-green-500' : 'bg-red-500'} rounded`} />
              <span>{status}</span>
            </div> */}
            <div className="flex items-center gap-2">
              <div
                className={`w-2 h-2 ${drone.status.connection.color} rounded`}
              />
              <span>{drone.status.connection.text}</span>
            </div>
            <div className="flex items-center gap-2">
              <div className={`w-2 h-2 ${drone.status.armed.color} rounded`} />
              <span>{drone.status.armed.text}</span>
            </div>
            <div className="flex items-center gap-2">
              <div className={`w-2 h-2 ${drone.status.flight.color} rounded`} />
              <span>{drone.status.flight.text}</span>
            </div>
            <div className="flex items-center gap-2">
              {/* <img
                className="w-[13px] h-1.5"
                alt="Group"
                src={`/group${
                  drone.id === 1 ? "" : drone.id === 2 ? "-1" : "-3"
                }.png`}
              /> */}
              <img className="w-3 h-3" alt="Vector" src="./vector.svg" />
              <span>{drone.status.time}</span>
            </div>

            <div className="flex items-center gap-2">
              <img
                className="w-[15px] h-2.5"
                alt="Vector"
                src="./Group.svg"
              />
              <span>{drone.status.Bettry}</span>
            </div>

            <div className="flex items-center gap-2">
              <img
                className="w-[13px] h-2.5"
                alt="Vector"
                src="./vector-3.svg"
              />
              <span>{drone.status.speed}</span>
            </div>

            {/* Unused Code i commented out i dont know this UI can used in future */}
            {/* <div className="flex items-center gap-2">
              <div className="flex space-x-1">
                <img
                  className="w-3.5 h-3.5"
                  alt="Vector"
                  src="./vector-10.svg"
                />
                <img className="w-3 h-3" alt="Vector" src="./vector.svg" />
                <img className="w-3.5 h-3.5" alt="Vector" src="./vector-8.svg" />
                
              </div>
            </div> */}
          </div>
        </div>
      </CardContent>
    </Card>
  );
};
