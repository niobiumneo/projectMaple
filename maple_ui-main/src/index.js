import React, { useEffect, useState } from 'react';
import ReactDOM from 'react-dom/client';
import ROSLIB from 'roslib';

const root = ReactDOM.createRoot(document.getElementById('root'));

function App() {
  const [connectionStatus, setConnectionStatus] = useState('Disconnected');
  const [ros, setRos] = useState(null);

  const [mapleAction, setMapleAction] = useState(null);       // /maple_action (JSON)
  const [mapleExpr, setMapleExpr] = useState(null);           // /maple_expression (String)
  const [mapleAppearance, setMapleAppearance] = useState(null); // /maple_appearance (String)

  const [currentScenario, setCurrentScenario] = useState(null);
  const [currentStateIndex, setCurrentStateIndex] = useState(0);
  const [feedbackAudio, setFeedbackAudio] = useState(null);

  // 1) Connect to rosbridge
  useEffect(() => {
    const rosInstance = new ROSLIB.Ros({
      // tip: set this via env if you want: process.env.REACT_APP_ROSBRIDGE_URL
      url: 'ws://192.168.10.101:9090'
    });

    rosInstance.on('connection', () => {
      console.log('Connected to ROS');
      setConnectionStatus('Connected');
      setRos(rosInstance);

      // Create topics once
      setMapleAction(new ROSLIB.Topic({
        ros: rosInstance,
        name: '/maple_action',
        messageType: 'std_msgs/String',
        queue_size: 1,
      }));
      setMapleExpr(new ROSLIB.Topic({
        ros: rosInstance,
        name: '/maple_expression',
        messageType: 'std_msgs/String',
        queue_size: 1,
      }));
      setMapleAppearance(new ROSLIB.Topic({
        ros: rosInstance,
        name: '/maple_appearance',
        messageType: 'std_msgs/String',
        queue_size: 1,
      }));
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
      try { rosInstance.close(); } catch (_) {}
    };
  }, []);

  // 2) Load scenario JSON
  useEffect(() => {
    fetch('/classroom_interaction/scenario_config.json')
      .then((response) => {
        if (!response.ok) throw new Error('Network response was not ok');
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

  // 3) On state change, publish one combined action to the orchestrator
  useEffect(() => {
    if (!ros || !currentScenario) return;
    if (currentStateIndex >= currentScenario.states.length) return;

    const state = currentScenario.states[currentStateIndex];

    // Build payload for /maple_action
    const payload = {};
    if (state.motion)      payload.motion = state.motion;                // e.g., "wave"
    if (state.tts)         payload.tts = state.tts;                      // e.g., "intro1" (key under pylips_phrases)
    if (state.expression)  payload.expression = state.expression;        // "HAPPY" | "SAD" | {AU...}
    if (state.appearance)  payload.appearance = state.appearance;        // "PRETTY" | {...}
    if (state.face_ms)     payload.face_ms = state.face_ms;              // ms (defaults handled by orchestrator)
    if (state.sync)        payload.sync = state.sync;                    // "parallel" | "speech_then_motion" | "motion_then_speech"
    if (state.wait_speech !== undefined) payload.wait_speech = !!state.wait_speech;

    if (mapleAction && Object.keys(payload).length > 0) {
      mapleAction.publish(new ROSLIB.Message({ data: JSON.stringify(payload) }));
      console.log('Published /maple_action', payload);
    }

    // Handle timed transition (unchanged)
    if (state.transition && state.transition.type === 'time') {
      const timeoutId = setTimeout(() => {
        setCurrentStateIndex((prevIndex) => prevIndex + 1);
      }, state.transition.duration);
      return () => clearTimeout(timeoutId);
    }
  }, [ros, currentScenario, currentStateIndex, mapleAction]);

  // 4) When user answers: drive face feedback with /maple_expression (HAPPY/SAD)
  const handleOptionClick = (selectedOption) => {
    const state = currentScenario.states[currentStateIndex];
    const correct = selectedOption === state.answer;

    if (mapleExpr) {
      mapleExpr.publish(new ROSLIB.Message({ data: correct ? 'HAPPY' : 'SAD' }));
      console.log(`Published /maple_expression: ${correct ? 'HAPPY' : 'SAD'}`);
    }

    // Optional: also send a quick action (e.g., nod motion + short face)
    // if (mapleAction) {
    //   mapleAction.publish(new ROSLIB.Message({
    //     data: JSON.stringify({
    //       motion: correct ? 'nod' : 'shake',
    //       expression: correct ? 'HAPPY' : 'SAD',
    //       face_ms: 1000,
    //       sync: 'parallel'
    //     })
    //   }));
    // }

    // Local feedback sounds (browser mp3s), same as before
    setFeedbackAudio(correct ? '/CORRECT.mp3' : '/INCORRECT.mp3');

    if (correct) {
      setTimeout(() => {
        setFeedbackAudio(null);
        setCurrentStateIndex((prevIndex) => prevIndex + 1);
      }, 3000);
    } else {
      setTimeout(() => setFeedbackAudio(null), 3000);
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
        {/* Connection pill (handy for debugging) */}
        <div style={{
          position:'fixed', top:12, right:12, padding:'6px 10px',
          borderRadius:8, fontSize:12, background:'#111', color:'#fff', opacity:0.8
        }}>
          ROS: {connectionStatus}
        </div>

        {currentScenario && currentStateIndex < currentScenario.states.length && (
          <>
            {currentScenario.states[currentStateIndex].image && (
              <img
                src={`/${currentScenario.scenario_name}/img/${currentScenario.states[currentStateIndex].image}`}
                alt="state"
                style={{ maxWidth: '100%', maxHeight: '60vh', margin: '20px', objectFit: 'contain' }}
              />
            )}

            {currentScenario.states[currentStateIndex].text && (
              <p style={{ fontSize: '1.5em', margin: '20px 0', fontWeight: 'bold' }}>
                {currentScenario.states[currentStateIndex].text}
              </p>
            )}

            {/* Browser-side audio (unchanged). This is separate from the orchestrator's TTS/wav streaming. */}
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
                    style={{ padding: '10px 20px', margin: '10px', fontSize: '1em', cursor: 'pointer' }}
                  >
                    {option}
                  </button>
                ))}
              </div>
            )}

            {feedbackAudio && <audio src={feedbackAudio} autoPlay />}
          </>
        )}

        {/* Optional: show the face UI inside your app */}
        {/* <iframe
          title="Maple Face"
          src="http://127.0.0.1:8000/face/maple"
          style={{ position:'fixed', right: 20, bottom: 20, width: 360, height: 270, border: 'none', borderRadius: 12 }}
        /> */}
      </header>
    </div>
  );
}

root.render(
  <React.StrictMode>
    <App />
  </React.StrictMode>
);
