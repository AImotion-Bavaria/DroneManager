import react from "@vitejs/plugin-react";
import tailwind from "tailwindcss";
import os from "node:os";
import { defineConfig } from "vite";

function getPreferredLanHost() {
  const interfaces = os.networkInterfaces();
  const ipv4Addresses = Object.values(interfaces)
    .flat()
    .filter((network) => network && network.family === "IPv4" && !network.internal)
    .map((network) => network.address);

  return (
    ipv4Addresses.find((address) => address.startsWith("192.")) ||
    ipv4Addresses[0] ||
    "0.0.0.0"
  );
}

const lanHost = getPreferredLanHost();

// https://vite.dev/config/
export default defineConfig({
  plugins: [react()],
  base: "./",
  server: {
    host: lanHost,
  },
  preview: {
    host: lanHost,
  },
  css: {
    postcss: {
      plugins: [tailwind()],
    },
  },
});
