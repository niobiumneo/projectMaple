import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';

function App() {
  const [rosConnection, setRosConnection] = useState(null);
  const [message, setMessage] = useState('');

  useEffect(() => {
    const ros = new ROSLIB.Ros({
      url: 'ws://localhost:9090'
    });

    ros.on('connection', () => {
      console.log('Connected to ROS');
      setRosConnection(ros);
    });

    ros.on('error', (error) => {
      console.log('Error connecting to ROS: ', error);
    });

    ros.on('close', () => {
      console.log('Connection to ROS closed');
    });

    return () => {
      if (rosConnection) {
        rosConnection.close();
      }
    };
  }, []);

  const handleClick = () => {
    if (rosConnection) {
      const pub = new ROSLIB.Topic({
        ros: rosConnection,
        name: '/motion_command',
        messageType: 'std_msgs/String'
      });

      const msg = new ROSLIB.Message({
        data: 'Hello from React!'
      });

      pub.publish(msg);
      setMessage('Message sent!');
    }
  };

  return (
    <div className="App">
      <header className="App-header">
        <p>React and ROS Integration</p>
        <button onClick={handleClick}>Send Message</button>
        <p>{message}</p>
      </header>
    </div>
  );
}

export default App;
