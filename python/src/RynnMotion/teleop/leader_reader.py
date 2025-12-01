"""
Threaded Leader Data Reader for Real-time Teleoperation

This module provides a non-blocking interface for reading leader robot data
in real-time teleoperation control loops. It separates data acquisition from
control execution to prevent blocking I/O operations from affecting timing.

Only used for real robot interfaces that inherit from RobotInterfaceBase.
MuJoCo simulations use MujocoRobotInterface directly without threading.
"""

import threading
import time
import numpy as np
from typing import Optional, Any, Dict
import logging

from RynnMotion.core.robotinterface_base import RobotInterfaceBase


class LeaderDataReader:
    """
    Threaded data reader for leader robot interfaces in teleoperation.

    Runs a background thread to continuously read data from leader robot interfaces
    that inherit from RobotInterfaceBase, preventing blocking I/O in control loops.

    This eliminates "Control loop running behind by 0.002s" warnings by separating
    slow serial I/O (1-5ms) from fast control loops (10ms @ 100Hz).
    
    Example Usage:
        # 1. Create reader for real leader robot
        from interface.lerobot_interface import create_robot_interface
        leader_interface = create_robot_interface(name="so101", mode="real")
        leader_reader = LeaderDataReader(leader_interface, target_frequency=200)
        
        # 2. In control loop (non-blocking, <0.1ms)
        while True:
            start_time = time.time()
            
            # Get latest leader data (non-blocking)
            leader_positions = leader_reader.get_latest_data()
            if leader_positions is not None:
                # Send to follower robot
                follower_interface.set_joint_positions(leader_positions)
            
            # Control loop timing is preserved
            follower_interface.step()
            
            # Maintain 100Hz control frequency
            elapsed = time.time() - start_time
            if elapsed < 0.01:
                time.sleep(0.01 - elapsed)
        
        # 3. Cleanup (optional - auto cleanup on destruction)
        leader_reader.stop()
    
    Threading Architecture:
        - Background thread: Reads leader data at 200Hz (every 5ms)
        - Control thread: Gets cached data instantly via get_latest_data()
        - Data freshness: Latest data is always <5ms old
        - Thread safety: Uses RLock for safe concurrent access
    """
    
    def __init__(self,
                 leader_interface: RobotInterfaceBase,
                 target_frequency: float = 200.0):
        """
        Initialize the leader data reader.

        Args:
            leader_interface: Leader robot interface inheriting from RobotInterfaceBase
            target_frequency: Target reading frequency in Hz (default: 200Hz)
        """
        self.leader_interface = leader_interface
        self.target_frequency = target_frequency
        self.timestep = 1.0 / target_frequency
        
        # Threading components
        self.data_lock = threading.RLock()
        self.reader_thread = None
        self.stop_event = threading.Event()
        
        # Data storage
        self.latest_data = None
        self.last_update_time = 0
        
        # Statistics
        self.read_count = 0
        self.error_count = 0
        
        # Logger
        self.logger = logging.getLogger(f"{__name__}.LeaderDataReader")
        
        # Auto-start thread
        self.start()
    
    def start(self) -> None:
        """Start the background data reading thread."""
        if self.reader_thread is not None and self.reader_thread.is_alive():
            self.logger.warning("Reader thread already running")
            return
        
        self.stop_event.clear()
        self.reader_thread = threading.Thread(
            target=self._read_loop,
            name=f"LeaderReader-{id(self)}",
            daemon=True
        )
        self.reader_thread.start()
        self.logger.info(f"Started leader data reader thread at {self.target_frequency}Hz")
    
    def stop(self) -> None:
        """Stop the background data reading thread."""
        if self.reader_thread is None:
            return
        
        self.logger.info("Stopping leader data reader thread...")
        self.stop_event.set()
        
        if self.reader_thread.is_alive():
            self.reader_thread.join(timeout=2.0)
            if self.reader_thread.is_alive():
                self.logger.warning("Reader thread did not stop gracefully")
        
        self.reader_thread = None
        self.logger.info("Leader data reader thread stopped")
    
    def _read_loop(self) -> None:
        """Main reading loop for the background thread."""
        self.logger.info("Leader data reader thread started")
        
        while not self.stop_event.is_set():
            start_time = time.time()
            
            try:
                # Read data from leader interface
                data = self.leader_interface.get_joint_positions()
                
                if data is not None:
                    # Update shared data with thread safety
                    with self.data_lock:
                        self.latest_data = data.copy() if hasattr(data, 'copy') else data
                        self.last_update_time = start_time
                    
                    self.read_count += 1
                else:
                    self.error_count += 1
                        
            except Exception as e:
                self.error_count += 1
                self.logger.debug(f"Error reading leader data: {e}")
            
            # Maintain target frequency
            elapsed = time.time() - start_time
            sleep_time = self.timestep - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        self.logger.info("Leader data reader thread finished")
    
    def get_latest_data(self) -> Optional[np.ndarray]:
        """
        Get the most recent leader robot data (non-blocking).
        
        Returns:
            Latest leader joint positions or None if no data available
        """
        with self.data_lock:
            if self.latest_data is not None:
                return self.latest_data.copy()
            return None
    
    def get_stats(self) -> Dict[str, Any]:
        """Get reader statistics."""
        with self.data_lock:
            current_time = time.time()
            age = current_time - self.last_update_time if self.last_update_time > 0 else float('inf')
            
            return {
                'thread_running': self.reader_thread is not None and self.reader_thread.is_alive(),
                'read_count': self.read_count,
                'error_count': self.error_count,
                'error_rate': self.error_count / max(1, self.read_count),
                'last_update_age': age,
                'target_frequency': self.target_frequency
            }
    
    def is_healthy(self) -> bool:
        """Check if the reader is healthy and providing fresh data."""
        stats = self.get_stats()
        
        # Check if we have recent data (within 2 timesteps)
        max_age = 2.0 * self.timestep
        if stats['last_update_age'] > max_age:
            return False
        
        # Check error rate
        if stats['error_rate'] > 0.1:  # More than 10% errors
            return False
        
        # Check if thread is running
        if not stats['thread_running']:
            return False
        
        return True
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - cleanup."""
        self.stop()