import { create } from "zustand";
import { persist, createJSONStorage } from "zustand/middleware";
import { dronePositions } from "../data/droneData";
import { DronePosition } from "../components/drones/types";

interface ConnectionState {
  connected: boolean;
  droneId: string | null;
  setConnected: (connected: boolean, droneId?: string | null) => void;
  swapped: boolean;
  setSwapped: (swapped: boolean) => void;
  searchStarted: boolean;
  setSearchStarted: (searchStarted: boolean) => void;
  dronePoss: DronePosition[];
  setDronePoss: (dronePoss: DronePosition[]) => void;
  dmConnected: boolean;
  setDmConnected: (dmConnected: boolean) => void;
}

export const useConnectionStore = create<ConnectionState>()(
  persist(
    (set) => ({
      connected: false,
      droneId: null,
      swapped: false,
      searchStarted: false,
      dronePoss: dronePositions || [],
      dmConnected: false,
      setDronePoss: (dronePoss) => set({ dronePoss }),
      setSearchStarted: (searchStarted) => set({ searchStarted }),
      setSwapped: (swapped) => set({ swapped }),
      setConnected: (connected, droneId = null) =>
        set({
          connected,
          droneId: connected ? droneId : null,
        }),
      setDmConnected: (dmConnected) => set({ dmConnected }),
    }),
    {
      name: "drone-connection-state",
      storage: createJSONStorage(() => sessionStorage), // Use sessionStorage instead of localStorage
      partialize: (state) => ({
        connected: state.connected,
        droneId: state.droneId,
      }),
    }
  )
);
