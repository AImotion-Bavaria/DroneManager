import axios from "axios";

const apiUrl = import.meta.env.VITE_API_URL;

const api = axios.create({
  baseURL: apiUrl, // Change this to your API URL
  timeout: 0,
  headers: {
    "Content-Type": "application/json",
  },
});

export default api;
