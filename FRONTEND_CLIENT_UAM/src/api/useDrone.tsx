import { useMutation, useQuery, useQueryClient } from "@tanstack/react-query";
import api from "./api";
import { DroneInfo, DroneResponse } from "../types/drone";

// Hook for connecting to a drone
export const useConnectDrone = () => {
  const queryClient = useQueryClient();

  return useMutation({
    mutationFn: async ({
      drone_id,
      connection_string,
    }: {
      drone_id: string;
      connection_string: string;
    }): Promise<DroneResponse> => {
      const response = await api.post("drones/connect", {
        drone_id,
        connection_string,
      });
      return response.data;
    },
    onSuccess: () => {
      // Invalidate and refetch drones data when we successfully connect
      queryClient.invalidateQueries({ queryKey: ["drones"] });
    },
  });
};

// Hook for disconnecting a drone
export const useDisconnectDrone = () => {
  const queryClient = useQueryClient();

  return useMutation({
    mutationFn: async (droneId: string): Promise<DroneResponse> => {
      const response = await api.post(`drones/${droneId}/disconnect`);
      return response.data;
    },
    onSuccess: () => {
      // Invalidate and refetch drones data when we successfully disconnect
      queryClient.invalidateQueries({ queryKey: ["drones"] });
    },
  });
};

// Hook for getting all drones
export const useGetDrones = () => {
  return useQuery({
    queryKey: ["drones"],
    queryFn: async (): Promise<DroneInfo[]> => {
      const response = await api.get("drones");
      return response.data;
    },
  });
};

export const useFlyDrone = () => {
  const queryClient = useQueryClient();

  return useMutation({
    mutationFn: async ({
      droneId,
      data = {
        x: 10,
        y: 10,
        z: -5,
        yaw: 0,
      },
    }: {
      droneId: string;
      data?: {
        x: number;
        y: number;
        z: number;
        yaw: number;
      };
    }): Promise<DroneResponse> => {
      const response = await api.post(`drones/${droneId}/flyto`, data);
      return response.data;
    },
    onSuccess: () => {
      queryClient.invalidateQueries({ queryKey: ["drones"] });
    },
  });
};
