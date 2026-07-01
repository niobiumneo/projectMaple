#!/usr/bin/env python3
"""
Setup PyLips Server
This script sets up and starts the pylips server for robot face display.
"""

import subprocess
import sys
import time
import os

def check_pylips_installation():
    """Check if pylips is installed"""
    try:
        import pylips
        print("✓ PyLips is installed")
        return True
    except ImportError:
        print("✗ PyLips is not installed")
        return False

def install_pylips():
    """Install pylips if not already installed"""
    print("Installing PyLips...")
    try:
        subprocess.check_call([sys.executable, "-m", "pip", "install", "pylips"])
        print("✓ PyLips installed successfully")
        return True
    except subprocess.CalledProcessError:
        print("✗ Failed to install PyLips")
        return False

def start_pylips_server():
    """Start the pylips server"""
    print("Starting PyLips server...")
    try:
        # Start pylips server in background
        process = subprocess.Popen([
            sys.executable, "-m", "pylips.server"
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        # Wait a moment for server to start
        time.sleep(2)
        
        if process.poll() is None:
            print("✓ PyLips server started successfully")
            return process
        else:
            print("✗ Failed to start PyLips server")
            return None
            
    except Exception as e:
        print(f"✗ Error starting PyLips server: {e}")
        return None

def test_pylips_connection():
    """Test connection to pylips"""
    try:
        from pylips.speech import RobotFace
        from pylips.face import FacePresets
        
        print("Testing PyLips connection...")
        face = RobotFace(robot_name='TestRobot', voice_id=None)
        face.set_appearance(FacePresets.default)
        print("✓ PyLips connection test successful")
        return True
    except Exception as e:
        print(f"✗ PyLips connection test failed: {e}")
        return False

def main():
    print("=== PyLips Server Setup ===")
    
    # Check if pylips is installed
    if not check_pylips_installation():
        install_choice = input("PyLips is not installed. Install now? (y/n): ")
        if install_choice.lower() == 'y':
            if not install_pylips():
                print("Setup failed. Please install PyLips manually.")
                return
        else:
            print("Setup cancelled.")
            return
    
    # Test connection
    if not test_pylips_connection():
        print("PyLips connection test failed. Please check your installation.")
        return
    
    # Start server
    server_process = start_pylips_server()
    if server_process:
        print("\nPyLips server is running!")
        print("You can now run your robot face bridge.")
        print("\nTo stop the server, press Ctrl+C")
        
        try:
            # Keep the server running
            server_process.wait()
        except KeyboardInterrupt:
            print("\nStopping PyLips server...")
            server_process.terminate()
            server_process.wait()
            print("Server stopped.")

if __name__ == '__main__':
    main() 