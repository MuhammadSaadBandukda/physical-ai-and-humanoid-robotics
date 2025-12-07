import React, { createContext, useState, useEffect, useContext, ReactNode } from 'react';

// Defined in User Instruction T012: mode ('sim' | 'physical')
// 'sim' maps to RTX Workstation (Digital Twin)
// 'physical' maps to Jetson Edge Kit (Physical AI)
export type HardwareMode = 'sim' | 'physical';

interface HardwareContextType {
  mode: HardwareMode;
  setMode: (mode: HardwareMode) => void;
}

const HardwareContext = createContext<HardwareContextType | undefined>(undefined);

const STORAGE_KEY = 'physical-ai-textbook:hardware-mode';

export const HardwareProvider = ({ children }: { children: ReactNode }) => {
  // Default to 'sim' as requested (Safer for beginners)
  const [mode, setModeState] = useState<HardwareMode>('sim');
  const [isInitialized, setIsInitialized] = useState(false);

  // Load from localStorage on mount (client-side only)
  useEffect(() => {
    const savedMode = localStorage.getItem(STORAGE_KEY);
    if (savedMode === 'sim' || savedMode === 'physical') {
      setModeState(savedMode);
    }
    setIsInitialized(true);
  }, []);

  // Persist to localStorage whenever mode changes
  const setMode = (newMode: HardwareMode) => {
    setModeState(newMode);
    localStorage.setItem(STORAGE_KEY, newMode);
  };

  // Prevent hydration mismatch by rendering children only after init
  // or simple return (Docusaurus is SSG, so we need to be careful about hydration)
  // For this atomic task, we will just return the provider. 
  // Handling SSG hydration issues usually involves a separate "BrowserOnly" check 
  // or acknowledging that the server render will default to 'sim'.
  
  return (
    <HardwareContext.Provider value={{ mode, setMode }}>
      {children}
    </HardwareContext.Provider>
  );
};

export const useHardware = (): HardwareContextType => {
  const context = useContext(HardwareContext);
  if (context === undefined) {
    throw new Error('useHardware must be used within a HardwareProvider');
  }
  return context;
};
