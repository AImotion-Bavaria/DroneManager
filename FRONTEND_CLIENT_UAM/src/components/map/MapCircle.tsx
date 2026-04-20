// @ts-nocheck
import { Button } from "../../components/ui/button";
import { MapTypeSelector } from "./MapTypeSelector";
import { useState } from "react";
import { mapTypes } from "../../data/mapData";

const MapCircle = () => {
  const [activeMapTypes, setActiveMapTypes] = useState<MapType[]>(mapTypes);

  const handleMapTypeSelect = (id: number) => {
    setActiveMapTypes(
      activeMapTypes.map((mapType) => ({
        ...mapType,
        active: mapType.id === id,
      }))
    );
  };

  return (
    <div className="flex flex-col gap-0">
      {/* Map Type Selector */}
      {/* <div className="mb-2"> */}
      <MapTypeSelector
        mapTypes={activeMapTypes}
        onSelect={handleMapTypeSelect}
      />
      {/* </div> */}

      {/* Map Circle */}
      <div className="relative w-[300px] h-[300px]">
        <div className="absolute inset-0">
          <div className="relative h-full w-full overflow-hidden">
            <img
              className="absolute top-[10px] left-[10px]"
              alt="Map"
              src="./map1.png"
            />
            <div className="absolute z-10 bottom-[20px] right-[24px] flex flex-col gap-1 p-1 bg-[#1a1a1a80] backdrop-blur-md rounded-xl shadow-lg">
              <button className="inline-flex items-center justify-center whitespace-nowrap text-sm font-medium focus-visible:outline-none disabled:pointer-events-none disabled:opacity-50 hover:bg-[#ffffff20] bg-transparent rounded-[8px] border-0 h-8 w-8 transition-colors">
                <svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" className="lucide lucide-plus w-5 h-5 text-white">
                  <path d="M5 12h14"></path>
                  <path d="M12 5v14"></path>
                </svg>
              </button>
              <button className="inline-flex items-center justify-center whitespace-nowrap text-sm font-medium focus-visible:outline-none disabled:pointer-events-none disabled:opacity-50 hover:bg-[#ffffff20] bg-transparent rounded-[8px] border-0 h-8 w-8 transition-colors">
                <svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" className="lucide lucide-minus w-5 h-5 text-white">
                  <path d="M5 12h14"></path>
                </svg>
              </button>
            </div>
          </div>
        </div>
        <img
          className="absolute w-[40px] h-[40px] top-[80px] left-[100px]"
          alt="Location Pin"
          src="./pin1.png"
        />
      </div>

      {/* Location Button */}
      <div className="flex items-center ml-2">
        <Button
          variant="outline"
          className="flex items-start gap-4 h-[36px] w-[270px] px-3 bg-[#e8e8e85d] rounded-[8px] border-none backdrop-blur-[24px] hover:bg-[#3d3d3d] transition-colors"
        >
          <img className="w-4 h-4" alt="Mission" src="./vector-4.svg" />
          <span className="text-[#dbdbdb] text-xs font-medium">
            Esplanade 10, building G001
          </span>
        </Button>
      </div>
    </div>
  );
};

export default MapCircle;
