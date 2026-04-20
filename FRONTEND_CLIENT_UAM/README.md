# Drone Management UI - Frontend

This is the frontend for the Drone Management System, built using modern web technologies. It provides a user interface for monitoring and controlling drones, visualizing their positions, and managing drone swaps.

## Tech Stack

- React (with TypeScript)
- Vite
- Tailwind CSS
- React Query (for API interactions)
- Zustand (for state management)
- Lucide React (for icons)
- Radix UI (for accessible UI components)

## Installation

Follow these steps to set up the frontend application:

1. **Clone the repository and navigate to the frontend directory**
    ```bash
    cd frontend
    ```

2. **Install dependencies**
    ```bash
    npm install
    ```

3. **Set up environment variables**
    
    Create a `.env.local` file in the root directory with the following values:
    ```
    VITE_API_URL=http://localhost:8000/api
   VITE_WS_URL=ws://localhost:8765
    VITE_DEFAULT_DRONE_NAME=drone1
    VITE_DEFAULT_UDP=udp://:14540
    ```

## Running the Application

1. **Start the development server**
    ```bash
    npm run dev
    ```

2. **Access the application**
    
    Open your browser and navigate to [http://localhost:5173](http://localhost:5173)

3. **Build for production**
    ```bash
    npm run build
    ```

## Prototype Mode

The application is designed to work in two modes:

1. **Connected Mode**: Connects to the backend API described in the documentation to control real drones.
2. **Prototype Mode**: If the connection to the actual drone or Gazebo simulator fails, the application automatically falls back to using static data from the frontend files.

This approach was taken due to potential issues with connecting to Gazebo in some environments. The prototype mode ensures that users can still experience the full UI and workflow even without a functional drone connection.

## Features

- **Drone Connection**: Connect to drones via the backend API
- **Drone Monitoring**: View drone status, position, and telemetry data
- **Map View**: Visualize drone positions in a map interface
- **Drone Swap**: Simulate drone swap operations when battery is low
- **Responsive Design**: Works on both desktop and mobile devices

## Key Components

### Connection Store

Located in `src/stores/connection-store.tsx`, this Zustand store manages the connection state and drone positions:

```typescript
// Key features of the connection store
export const useConnectionStore = create<ConnectionState>()(
  persist(
     (set) => ({
        connected: false,
        droneId: null,
        swapped: false,
        searchStarted: false,
        dronePoss: dronePositions || [],
        setDronePoss: (dronePoss) => set({ dronePoss }),
        setSearchStarted: (searchStarted) => set({ searchStarted }),
        setSwapped: (swapped) => set({ swapped }),
        setConnected: (connected, droneId = null) =>
          set({
             connected,
             droneId: connected ? droneId : null,
          }),
     }),
     {
        name: "drone-connection-state",
        storage: createJSONStorage(() => sessionStorage),
     }
  )
);
```

### API Integration

The application uses React Query to interact with the backend API:

```typescript
// Example of API integration for connecting to a drone
const { mutate: connectDrone, isPending: isConnecting } = useConnectDrone();

// Usage
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
        // Handle connection errors
     },
  }
);
```

### Fallback to Static Data

If the connection to the real drone fails, the application uses static data from `src/data/droneData.ts`:

```typescript
// Sample static drone data
export const droneData = [
  {
     id: 1,
     name: "John",
     image: "/image1.png",
     status: {
        connection: { text: "Connected", color: "bg-green-500" },
        armed: { text: "Armed", color: "bg-green-500" },
        flight: { text: "In Air", color: "bg-green-500" },
        time: "50 mins",
        speed: "12km/h",
     },
  },
  // More drones...
];
```

## Troubleshooting

- If you can't connect to the backend API, check that the backend server is running and that your `.env.local` file has the correct API URL.
- If you experience issues with the Gazebo simulator, the application will automatically use static data to demonstrate functionality.
- For connection issues, verify the UDP port settings in the connection dialog.

## Development

The project structure follows a component-based architecture with the following organization:

- `/src/components`: Reusable UI components
- `/src/screens`: Main application screens
- `/src/stores`: State management with Zustand
- `/src/api`: API integration with React Query
- `/src/data`: Static data for prototype mode
- `/src/lib`: Utility functions and helpers

When extending the application, consider adding new components to the appropriate directory and updating the state management as needed.