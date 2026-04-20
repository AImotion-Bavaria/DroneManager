import { Card } from "../../components/ui/card";
import { MapType } from "./types";
import { Button } from "../../components/ui/button";


interface MapTypeSelectorProps {
  mapTypes: MapType[];
  onSelect?: (id: number) => void;
}

export const MapTypeSelector = ({
  mapTypes,
  onSelect,
}: MapTypeSelectorProps): JSX.Element => {
  return (
    <div className="flex gap-4 mb-2 ml-[15px]">
      {mapTypes.map((mapType) => (
        <>
        <Card
          key={mapType.id}
          className={`w-[78px] h-[84px] bg-[#d9d9d94c] rounded-[10px] ${
            mapType.active ? "border-2 border-solid border-[#55cde2]" : ""
          } backdrop-blur-[7px] relative cursor-pointer`}
          onClick={() => onSelect && onSelect(mapType.id)}
        >
          <img
            className="absolute top-1/2 left-1/2 transform -translate-x-1/2 -translate-y-[60%] w-[80%] h-auto object-cover"
            alt={mapType.name}
            src={mapType.image}
          />
          <span className="absolute bottom-0 left-1/2 transform -translate-x-1/2 text-white text-xs">
            {mapType.name}
          </span>
        </Card>
        
        </>
        
        
      ))}
    </div>
  );
};
