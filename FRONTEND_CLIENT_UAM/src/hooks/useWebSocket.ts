// src/hooks/useWebSocket.ts
import { useState, useEffect, useCallback, useRef } from 'react';

 interface DroneData {
  drones: {
    [key: string]: {
      position: number[];
      conn: boolean;
      armed: boolean;
      in_air: boolean;
      
      // Fields from Mock Scripts
      speed?: number;
      heading?: number;
      
      // Fields from Real DroneManager
      velocity?: number[];
      attitude?: number[];
      mode?: string;
      gps?: number[];
      rtsp?: string;
      fence?: any[];
      target?: any[];
    };
  };
  missions?: {
    [missionName: string]: {
      stage?: string;
      drones?: string[];
     bat?: Record<string, number>; // battery percentages by drone name
    };
  };
  timestamp: number;
}

export const useWebSocket = () => {
  const [isConnected, setIsConnected] = useState(false);
  const [droneData, setDroneData] = useState<DroneData | null>(null);
  const [simulatedBatteries, setSimulatedBatteries] = useState<Record<string, number>>({});
  const wsRef = useRef<WebSocket | null>(null);
  const reconnectTimeoutRef = useRef<number | null>(null);
  const shouldReconnectRef = useRef(true);

  const configuredUrl = import.meta.env.VITE_WS_URL?.trim();
  const currentHostname =
    typeof window !== 'undefined' && window.location.hostname
      ? window.location.hostname
      : 'localhost';
  const configuredLanHost = import.meta.env.VITE_LAN_HOST?.trim();
  const hostname = currentHostname.startsWith('192.')
    ? currentHostname
    : configuredLanHost?.startsWith('192.')
      ? configuredLanHost
      : currentHostname;
  const protocol =
    typeof window !== 'undefined' && window.location.protocol === 'https:'
      ? 'wss'
      : 'ws';
  const wsUrl = configuredUrl || `${protocol}://${hostname}:8765`;

  const droneDataRef = useRef<DroneData | null>(null);

  const connect = useCallback(() => {
    if (!shouldReconnectRef.current) {
      return;
    }

    if (
      wsRef.current &&
      (wsRef.current.readyState === WebSocket.OPEN ||
        wsRef.current.readyState === WebSocket.CONNECTING)
    ) {
      return;
    }

    try {
      const websocket = new WebSocket(wsUrl);
      wsRef.current = websocket;
      
      websocket.onopen = () => {
        console.log(`WebSocket connected: ${wsUrl}`);
        setIsConnected(true);
      };

      websocket.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          setDroneData(data);
          droneDataRef.current = data;
        //   console.log('Received data:', data);

        // can you please open the chrome here ???
        } catch (error) {
          console.error('Error parsing WebSocket data:', error);
        }
      };

      websocket.onclose = () => {
        console.log('WebSocket disconnected');
        setIsConnected(false);
        wsRef.current = null;

        if (shouldReconnectRef.current) {
          reconnectTimeoutRef.current = window.setTimeout(connect, 2000);
        }
      };

      websocket.onerror = (error) => {
        console.error('WebSocket Error:', error);
        websocket.close();
      };
    } catch (error) {
      console.error('Connection error:', error);

      if (shouldReconnectRef.current) {
        reconnectTimeoutRef.current = window.setTimeout(connect, 2000);
      }
    }
  }, [wsUrl]);

  useEffect(() => {
    shouldReconnectRef.current = true;
    connect();

    return () => {
      shouldReconnectRef.current = false;

      if (reconnectTimeoutRef.current !== null) {
        window.clearTimeout(reconnectTimeoutRef.current);
        reconnectTimeoutRef.current = null;
      }

      if (wsRef.current) {
        wsRef.current.close();
        wsRef.current = null;
      }
    };
  }, [connect]);

  useEffect(() => {
    const interval = setInterval(() => {
      setSimulatedBatteries(prev => {
        const next = { ...prev };
        let updated = false;
        const currentData = droneDataRef.current;
        if (currentData?.drones) {
          // --- Custom logic for yellow low battery warning ---
          // Find if yellow is the only one in air and stage is SearchGroup
          const drones = currentData.drones;
          const droneNames = Object.keys(drones);
          const yellowInAir = drones.yellow?.in_air;
          const yellowBattery = (currentData.missions?.uam?.bat?.yellow ?? next["yellow"]);
          const yellowIsOnlyInAir = yellowInAir && droneNames.every(name => {
            if (name === 'yellow') return true;
            return !drones[name]?.in_air;
          });
          const stage = currentData.missions?.uam?.stage;
          // If yellow is the only one in air, battery > 0.50, and stage is SearchGroup, drop it fast to just below 0.50
          if (yellowIsOnlyInAir && yellowBattery > 0.50 && stage === 'SearchGroup') {
            next["yellow"] = 0.49;
            updated = true;
          }
          // --- End custom logic ---
          Object.entries(currentData.drones).forEach(([name, data]) => {
            if (!(name in next) || next[name] === 1.0) {
              next[name] = 0.9; // Always start simulated battery at 90%
              updated = true;
            }
            if (data.in_air) {
              // Drain: 100% to 0% in 1800s (30 minutes)
              // Minimum level: 0.165 (critical_level / 2)
              // If yellow, and we just forced it below 0.50, continue normal drain from there
              const drainFrom = next[name];
              const newLevel = Math.max(0.33 / 2, drainFrom - (1 / 1800));
              if (newLevel !== next[name]) {
                next[name] = newLevel;
                updated = true;
              }
            } else {
              // Reset to 0.9 when not in air
              if (next[name] !== 0.9) {
                next[name] = 0.9;
                updated = true;
              }
            }
          });
        }
        return updated ? next : prev;
      });
    }, 1000);
    return () => clearInterval(interval);
  }, []); // Run only once

  const sendMessage = useCallback((message: any) => {
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify(message));
    }
  }, []);

  // Inject simulated batteries into the missions payload if they are missing
  // Also, add a parallel object to indicate which batteries are fake
  let enrichedDroneData = null;
  if (droneData) {
    const realBat = (droneData.missions?.uam?.bat) || {};
    const bat: Record<string, number> = { ...simulatedBatteries, ...realBat };
    const batIsFake: Record<string, boolean> = {};
    Object.keys(bat).forEach(name => {
      // If the value comes from simulatedBatteries and not from realBat, it's fake
      batIsFake[name] = !(name in realBat);
    });
    enrichedDroneData = {
      ...droneData,
      missions: {
        ...(droneData.missions || {}),
        uam: {
          ...(droneData.missions?.uam || {}),
          bat,
          batIsFake
        }
      }
    };
  }

  // Debugging log for the user
  if (enrichedDroneData) {
    console.log("Enriched Data Battery (uam):", enrichedDroneData.missions?.uam?.bat);
  }

  return { isConnected, droneData: enrichedDroneData, sendMessage };
};