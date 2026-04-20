import useSWRMutation from "swr/mutation";
import api from "./api";

// Define a generic function for POST requests
const postFetcher = async <T, R>(url: string, { arg }: { arg: T }) => {
  const response = await api.post<T>(url, arg);
  console.log(response)
  return response.data as unknown as R;
};

// Custom hook for POST requests
export const usePost = <R, T>(url: string) => {
  const { trigger, data, error, isMutating } = useSWRMutation<R, Error, string, T>(url, postFetcher<T, R>);

  return { trigger, data, error, isMutating };
};
