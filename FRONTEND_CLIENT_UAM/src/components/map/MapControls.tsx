import { MinusIcon, PlusIcon } from "lucide-react";
import { Button } from "../../components/ui/button";

interface MapControlsProps {
  onZoomIn?: () => void;
  onZoomOut?: () => void;
}

export const MapControls = ({
  onZoomIn,
  onZoomOut,
}: MapControlsProps): JSX.Element => {
  return (
    <div className="flex flex-col rounded-xl overflow-hidden shadow-lg w-10 max-h-[80px]">
      <Button
        variant="ghost"
        className="h-10 hover:bg-[#ffffff20] bg-[#1a1a1a80] rounded-none border-0 border-b border-[#ffffff20] p-0 transition-colors"
        onClick={onZoomIn}
      >
        <PlusIcon className="w-5 h-5 text-white" />
      </Button>
      <Button
        variant="ghost"
        className="h-10 hover:bg-[#ffffff20] bg-[#1a1a1a80] rounded-none border-0 p-0 transition-colors"
        onClick={onZoomOut}
      >
        <MinusIcon className="w-5 h-5 text-white" />
      </Button>
    </div>
  );
};
