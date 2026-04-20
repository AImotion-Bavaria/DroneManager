import useSWR from "swr";
import api from "./api";

// Generic Type for API Response
export const useFetch = <T>(url: string) => {
  const fetcher = (url: string) => api.get<T>(url).then((res) => res.data);

  const { data, error, isLoading } = useSWR<T>(url, fetcher);

  return {
    data,
    error,
    isLoading,
  };
};
