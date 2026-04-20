import { DroneData, DronePosition } from "../components/drones/types";

// Data for drone cards
export const droneData: DroneData[] = [
  {
    id: 1,
    name: "Drone1",
    image: "image.png",
    status: {
      connection: { text: "Connected", color: "bg-[#8bf600]" },
      armed: { text: "Armed", color: "bg-[#55cde2]" },
      flight: { text: "In Air", color: "bg-[#55cde2]" },
      time: "50 mins",
      speed: "13km/h",
    },
  },
  {
    id: 2,
    name: "Luke",
    image: "image1.png",
    status: {
      connection: { text: "Connected", color: "bg-[#8bf600]" },
      armed: { text: "Armed", color: "bg-[#55cde2]" },
      flight: { text: "In Air", color: "bg-[#55cde2]" },
      time: "50 mins",
      speed: "13km/h",
    },
  },
  {
    id: 3,
    name: "Tom",
    image: null, // No image, using dark background
    status: {
      connection: { text: "Disconnected", color: "bg-[#f64e00]" },
      armed: { text: "Armed", color: "bg-[#f64e00]" },
      flight: { text: "In Air", color: "bg-[#f64e00]" },
      time: "50 mins",
      speed: "0km/h",
    },
  },
];

// Updated drone positions with responsive classes
export const dronePositions: DronePosition[] = [
  {
    id: 1,
    className: "top-[25%] left-[45%] md:top-[25%] md:left-[20%]",
    type: "active",
    movedClassName: "top-[25%] left-[95%] md:top-[25%] md:left-[80%]",
    pulse:false,
  },
  {
    id: 2,
    className: "top-[30%] left-[45%] md:top-[39%] md:left-[20%]",
    type: "active",
    movedClassName: "top-[5%] left-[95%] md:top-[25%] md:left-[80%]",
    pulse:false,
  },
  // {
  //   id: 3,
  //   className: "top-[40%] left-[45%] md:top-[52%] md:left-[20%]",
  //   type: "standard",
  //   movedClassName: "top-[25%] left-[95%] md:top-[25%] md:left-[80%]",
  //   pulse:false,
  // },
];