import React, { useEffect, useState } from 'react';
import ReactDOM from 'react-dom/client';
import ROSLIB from 'roslib';

const root = ReactDOM.createRoot(document.getElementById('root'));

function App() {
  const [connectionStatus, setConnectionStatus] = useState('Disconnected');
  const [ros, setRos] = useState(null);
  const [currentScenario, setCurrentScenario] = useState(null);
  const [currentStateIndex, setCurrentStateIndex] = useState(0);
  const [feedbackAudio, setFeedbackAudio] = useState(null);

  useEffect(() => {
    const rosInstance = new ROSLIB.Ros({
      url: 'ws://192.168.10.101:9090'  
    });

    rosInstance.on('connection', () => {
      console.log('Connected to ROS');
      setConnectionStatus('Connected');
      setRos(rosInstance); 
    });

    rosInstance.on('error', (error) => {
      console.error('Error connecting to ROS:', error);
      setConnectionStatus('Error');
    });

    rosInstance.on('close', () => {
      console.log('Connection to ROS closed');
      setConnectionStatus('Disconnected');
    });

    return () => {
      if (rosInstance) {
        rosInstance.close();
      }
    };
  }, []);  

  useEffect(() => {
    fetch('/classroom_interaction/scenario_config.json')
      .then((response) => {
        if (!response.ok) {
          throw new Error('Network response was not ok');
        }
        return response.json();
      })
      .then((data) => {
        setCurrentScenario(data);
        setCurrentStateIndex(0);
      })
      .catch((error) => {
        console.error('Error loading scenario JSON:', error);
      });
  }, []);

  useEffect(() => {
    if (ros && currentScenario && currentStateIndex < currentScenario.states.length) {
      const state = currentScenario.states[currentStateIndex];
  
      // Handle nodding during city descriptions
      if (state.motion === 'nod') {
        const topic = new ROSLIB.Topic({
          ros: ros,
          name: '/motion_command',
          messageType: 'std_msgs/String'
        });
  
        topic.publish(new ROSLIB.Message({
          data: 'nod'
        }));
      }
  
      // Handle thinking and guessing for questions
      if (state.options) {
        console.log('Question state detected, preparing robot response...');
        
        // First show thinking face
        const faceTopic = new ROSLIB.Topic({
          ros: ros,
          name: '/pylips_face',
          messageType: 'std_msgs/String'
        });
        
        faceTopic.publish(new ROSLIB.Message({
          data: 'default'
        }));
        
        // Show thinking motion
        const motionTopic = new ROSLIB.Topic({
          ros: ros,
          name: '/motion_command',
          messageType: 'std_msgs/String'
        });
  
        motionTopic.publish(new ROSLIB.Message({
          data: 'think'
        }));
  
        // After a short delay, robot makes a guess
        setTimeout(() => {
          // Randomly select an option (except the correct answer)
          const wrongOptions = state.options.filter(opt => opt !== state.answer);
          const randomGuess = wrongOptions[Math.floor(Math.random() * wrongOptions.length)];
          
          // Use Pylips to say the thinking phrase
          const ttsTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/pylips_tts',
            messageType: 'std_msgs/String'
          });
          
          const thinkingPhrase = `Hmm, I think it should be ${randomGuess}...`;
          console.log('Speaking thinking phrase:', thinkingPhrase);
          
          ttsTopic.publish(new ROSLIB.Message({
            data: thinkingPhrase
          }));
        }, 2000);
      }
  
      if (state.transition.type === 'time') {
        const timeoutId = setTimeout(() => {
          setCurrentStateIndex((prevIndex) => prevIndex + 1);
        }, state.transition.duration);
  
        return () => clearTimeout(timeoutId);
      }
    }
  }, [ros, currentScenario, currentStateIndex]);
  
  const handleOptionClick = (selectedOption) => {
    const state = currentScenario.states[currentStateIndex];
    const correct = selectedOption === state.answer;
  
    if (ros) {
      // Update robot's face expression
      const faceTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/pylips_face',
        messageType: 'std_msgs/String'
      });
      
      faceTopic.publish(new ROSLIB.Message({
        data: correct ? 'happy' : 'sad'
      }));
      
      // Update robot's motion
      const motionTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/motion_command',
        messageType: 'std_msgs/String'
      });
  
      motionTopic.publish(new ROSLIB.Message({
        data: correct ? 'happy' : 'sad'
      }));
    }
  
    setFeedbackAudio(correct ? '/CORRECT.mp3' : '/INCORRECT.mp3');
  
    if (correct) {
      setTimeout(() => {
        setFeedbackAudio(null); 
        setCurrentStateIndex((prevIndex) => prevIndex + 1);
      }, 3000); 
    } else {
      setTimeout(() => {
        setFeedbackAudio(null); 
      }, 3000); 
    }
  };
  
  return (
    <div className="App">
      <header
        className="App-header"
        style={{
          display: 'flex',
          justifyContent: 'center',
          alignItems: 'center',
          minHeight: '100vh',
          textAlign: 'center', 
          flexDirection: 'column' 
        }}
      >
        {currentScenario && currentStateIndex < currentScenario.states.length && (
          <>
            {
              currentScenario.states[currentStateIndex].image && (
                <img
                  src={`/${currentScenario.scenario_name}/img/${currentScenario.states[currentStateIndex].image}`}
                  alt="state"
                  style={{
                    maxWidth: '100%',    
                    maxHeight: '60vh',  
                    margin: '20px',       
                    objectFit: 'contain', 
                  }}
                />
              )
            }
            {currentScenario.states[currentStateIndex].text && (
              <p style={{ fontSize: '1.5em', margin: '20px 0', fontWeight: 'bold' }}>
                {currentScenario.states[currentStateIndex].text}
              </p>
            )}
            
            {currentScenario.states[currentStateIndex].audio && (
              <audio
                src={`/${currentScenario.scenario_name}/audio/${currentScenario.states[currentStateIndex].audio}`}
                autoPlay
              />
            )}

            {currentScenario.states[currentStateIndex].options && (
              <div style={{ marginTop: '20px' }}>
                {currentScenario.states[currentStateIndex].options.map((option, index) => (
                  <button
                    key={index}
                    onClick={() => handleOptionClick(option)}
                    style={{
                      padding: '10px 20px',
                      margin: '10px',
                      fontSize: '1em',
                      cursor: 'pointer'
                    }}
                  >
                    {option}
                  </button>
                ))}
              </div>
            )}

            {feedbackAudio && (
              <audio src={feedbackAudio} autoPlay />
            )}
          </>
        )}
      </header>
    </div>
  );
}

root.render(
  <React.StrictMode>
    <App />
  </React.StrictMode>
);
