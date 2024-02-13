import { useEffect, useState } from "react";
import { TiBatteryHigh } from "react-icons/ti";
import "./App.css";

function App() {
  const [energy, setEnergy] = useState(75); // 0 - 100
  useEffect(() => {
    setEnergy(75);
  }, []);
  return (
    <div className="flex flex-col justify-between items-center text-center w-[800px] h-[480px] p-10 bg-black text-white ">
      {/* header */}
      <div className="flex flex-row justify-between items-center w-full px-[150px]">
        <div className="flex flex-row justify-center items-center gap-2">
          <p className="">IMD</p>
          <div className="w-3 h-3 rounded-full bg-white"></div>
        </div>
        <div className="flex flex-row justify-center items-center gap-2">
          <p className="">PCC</p>
          <div className="w-3 h-3 rounded-full bg-white"></div>
        </div>
        <div>
          <p className="">60.4 A</p>
        </div>
      </div>
      {/* speed */}
      <div className="relative flex flex-col gap-5 border-solid border-2 rounded-full w-60 h-60 justify-center items-center">
        {/* <div className="absolute"> */}
        <p className="absolute left-2 text-xs text-center">0</p>
        <p className="absolute left-[34px] top-[44px] text-xs text-center">
          20
        </p>
        <p className="absolute left-[108px] top-[6px] text-xs text-center">
          40
        </p>
        <p className="absolute right-[34px] top-[44px] text-xs text-center">
          60
        </p>
        <p className="absolute right-2 text-xs text-center">80</p>
        {/* </div> */}
        <div className="absolute flex flex-row items-end">
          <p className="text-6xl font-bold">17</p>
          <p className="text-base -ml-1">mph</p>
        </div>
      </div>
      {/* footer */}
      <div className="flex flex-col justify-between w-fit">
        {/* energy bar */}
        <div className="w-80 h-1 bg-cyan-400 bg-opacity-30 ">
          <div
            className="h-1 bg-cyan-400 shadow-2xl shadow-cyan-400"
            style={{ width: `${(energy / 100) * 320}px` }}
          ></div>
        </div>
        {/* status numbers */}
        <div className="flex flex-row justify-between py-1">
          <div className="flex flex-row justify-center items-center gap-1">
            <TiBatteryHigh size={30} color="white" />
            <p>96%</p>
          </div>
          <p>100.1 V</p>
        </div>
      </div>
    </div>
  );
}

export default App;
