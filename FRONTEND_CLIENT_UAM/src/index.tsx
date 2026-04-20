import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import { Element } from "./screens/Element";
import { QueryClient, QueryClientProvider } from "@tanstack/react-query";
// Create a client
const queryClient = new QueryClient({
  defaultOptions: {
    queries: {
      refetchOnWindowFocus: false, // Don't refetch when window regains focus
      retry: 1, // Only retry failed requests once
    },
  },
});

createRoot(document.getElementById("app") as HTMLElement).render(
  <StrictMode>
    <QueryClientProvider client={queryClient}>
      <Element />
    </QueryClientProvider>
  </StrictMode>
);
