import { useState, useEffect } from 'react';

// Define the User Profile interface based on the Spec
export interface UserProfile {
  id: string;
  email: string;
  hardwareProfile: 'RTX' | 'Jetson';
  language: 'EN' | 'UR';
}

export const useUserProfile = () => {
  const [user, setUser] = useState<UserProfile | null>(null);
  const [loading, setLoading] = useState<boolean>(true);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    const fetchProfile = async () => {
      try {
        setLoading(true);
        
        // MOCK IMPLEMENTATION (Since Backend T006-T010 is deferred)
        // In a real scenario, this would be:
        // const response = await fetch('/api/user/profile');
        // const data = await response.json();
        
        // Simulating network delay
        await new Promise(resolve => setTimeout(resolve, 1000));

        // Mock Data
        const mockUser: UserProfile = {
          id: 'mock-user-123',
          email: 'student@university.edu',
          hardwareProfile: 'RTX', // This would normally come from the DB
          language: 'EN'
        };

        console.log('✅ [T015] Fetched User Profile (Mock):', mockUser);
        setUser(mockUser);
      } catch (err) {
        console.error('❌ [T015] Failed to fetch profile:', err);
        setError('Failed to load user profile');
      } finally {
        setLoading(false);
      }
    };

    fetchProfile();
  }, []);

  return { user, loading, error };
};
