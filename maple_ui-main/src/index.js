import React, { useEffect, useRef, useState } from 'react';
import ReactDOM from 'react-dom/client';
import ROSLIB from 'roslib';

const root = ReactDOM.createRoot(document.getElementById('root'));

function App() {
  const [connectionStatus, setConnectionStatus] = useState('Disconnected');
  const [ros, setRos] = useState(null);

  const [currentScenario, setCurrentScenario] = useState(null);
  const [currentStateIndex, setCurrentStateIndex] = useState(0);
  const [feedbackAudio, setFeedbackAudio] = useState(null);

  // Pause/resume state
  const [isPaused, setIsPaused] = useState(false);
  const [resumeNonce, setResumeNonce] = useState(0);

  // Timers and audio refs (browser audio)
  const transitionTimeoutRef = useRef(null);
  const feedbackTimeoutRef = useRef(null);
  const stateAudioRef = useRef(null);
  const feedbackAudioRef = useRef(null);

  // ROS topics (refs so they persist without re-render loops)
  const motionTopicRef = useRef(null);              // /motion_command
  const interactionControlRef = useRef(null);       // /interaction_control
  const mapleActionRef = useRef(null);              // /maple_action (JSON in std_msgs/String)
  const mapleExprRef = useRef(null);                // /maple_expression
  const mapleAppearanceRef = useRef(null);          // /maple_appearance

  // Prevent duplicate publishes in React StrictMode dev
  const lastPublishKeyRef = useRef('');

  const clearAllTimeouts = () => {
    if (transitionTimeoutRef.current) {
      clearTimeout(transitionTimeoutRef.current);
      transitionTimeoutRef.current = null;
    }
    if (feedbackTimeoutRef.current) {
      clearTimeout(feedbackTimeoutRef.current);
      feedbackTimeoutRef.current = null;
    }
  };

  const stopAllBrowserAudio = () => {
    if (stateAudioRef.current) {
      stateAudioRef.current.pause();
      stateAudioRef.current.currentTime = 0;
    }
    if (feedbackAudioRef.current) {
      feedbackAudioRef.current.pause();
      feedbackAudioRef.current.currentTime = 0;
    }
  };

  const publishInteractionControl = (cmd) => {
    const t = interactionControlRef.current;
    if (!t) return;
    t.publish(new ROSLIB.Message({ data: cmd }));
  };

  const publishMotion = (motionName) => {
    const t = motionTopicRef.current;
    if (!t || !motionName) return;
    t.publish(new ROSLIB.Message({ data: motionName }));
  };

  // This is the important part you lost: publish Maple orchestration payload
  const publishMapleActionForState = (state, { includeMotion = true } = {}) => {
    // Optional direct expression/appearance topics (useful even if orchestrator ignores them)
    if (state.expression && mapleExprRef.current) {
      mapleExprRef.current.publish(new ROSLIB.Message({ data: String(state.expression) }));
    }
    if (state.appearance && mapleAppearanceRef.current) {
      mapleAppearanceRef.current.publish(new ROSLIB.Message({ data: String(state.appearance) }));
    }

    // Main combined action payload
    const payload = {};
    if (includeMotion && state.motion) payload.motion = state.motion;

    // If your orchestrator expects a key like "intro1" to play via PyLips/TTS:
    if (state.tts) payload.tts = state.tts;

    // Some setups use expression/appearance inside maple_action too
    if (state.expression) payload.expression = state.expression;
    if (state.appearance) payload.appearance = state.appearance;

    if (state.face_ms) payload.face_ms = state.face_ms;
    if (state.sync) payload.sync = state.sync;
    if (state.wait_speech !== undefined) payload.wait_speech = !!state.wait_speech;

    if (mapleActionRef.current && Object.keys(payload).length > 0) {
      mapleActionRef.current.publish(new ROSLIB.Message({ data: JSON.stringify(payload) }));
      console.log('Published /maple_action', payload);
    }
  };

  const handlePause = () => {
    if (isPaused) return;
    setIsPaused(true);
    clearAllTimeouts();
    stopAllBrowserAudio();
    setFeedbackAudio(null);

    // Pause robot motion execution (robot_ctr_with_pause)
    publishInteractionControl('pause');

    // If you later add pause support in orchestrator, you could also publish there.
  };

  const handleResume = () => {
    if (!isPaused) return;
    setIsPaused(false);

    // Resume robot motion execution (robot_ctr_with_pause repeats current pose)
    publishInteractionControl('resume');

    // Restart browser audio + timers for this state
    setResumeNonce((n) => n + 1);

    // Re-trigger speech/face from the beginning of this state by re-publishing maple_action
    // But do NOT include motion here, because robot_ctr is already resuming motion.
    if (currentScenario && currentStateIndex < currentScenario.states.length) {
      const state = currentScenario.states[currentStateIndex];
      publishMapleActionForState(state, { includeMotion: false });
    }
  };

  // ROS connection
  useEffect(() => {
    const rosInstance = new ROSLIB.Ros({
      // best: always connect to the same host serving the UI
      // url: `ws://${window.location.hostname}:9090`,
      url: 'ws://192.168.10.101:9090',
    });

    rosInstance.on('connection', () => {
      console.log('Connected to ROS');
      setConnectionStatus('Connected');
      setRos(rosInstance);

      // Create topics once
      motionTopicRef.current = new ROSLIB.Topic({
        ros: rosInstance,
        name: '/motion_command',
        messageType: 'std_msgs/String',
        queue_size: 1,
      });

      interactionControlRef.current = new ROSLIB.Topic({
        ros: rosInstance,
        name: '/interaction_control',
        messageType: 'std_msgs/String',
        queue_size: 1,
      });

      mapleActionRef.current = new ROSLIB.Topic({
        ros: rosInstance,
        name: '/maple_action',
        messageType: 'std_msgs/String',
        queue_size: 1,
      });

      mapleExprRef.current = new ROSLIB.Topic({
        ros: rosInstance,
        name: '/maple_expression',
        messageType: 'std_msgs/String',
        queue_size: 1,
      });

      mapleAppearanceRef.current = new ROSLIB.Topic({
        ros: rosInstance,
        name: '/maple_appearance',
        messageType: 'std_msgs/String',
        queue_size: 1,
      });
    });

    rosInstance.on('error', (error) => {
      console.error('Error connecting to ROS:', error);
      setConnectionStatus('Error');
    });

    rosInstance.on('close', () => {
      console.log('Connection to ROS closed');
      setConnectionStatus('Disconnected');

      motionTopicRef.current = null;
      interactionControlRef.current = null;
      mapleActionRef.current = null;
      mapleExprRef.current = null;
      mapleAppearanceRef.current = null;
    });

    return () => {
      try {
        rosInstance.close();
      } catch (_) {}
    };
  }, []);

  // Load scenario JSON
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

  // On state entry: publish motion + maple_action and schedule timed transition
  useEffect(() => {
    clearAllTimeouts();

    if (!ros || !currentScenario) return;
    if (isPaused) return;
    if (currentStateIndex >= currentScenario.states.length) return;

    const state = currentScenario.states[currentStateIndex];

    // Avoid duplicate publishes caused by React StrictMode dev double-effect
    const key = `state-${currentStateIndex}-resume-${resumeNonce}`;
    if (lastPublishKeyRef.current !== key) {
      lastPublishKeyRef.current = key;

      // Move robot
      if (state.motion) publishMotion(state.motion);

      // Trigger robot speech + face (PyLips) via orchestrator topics
      publishMapleActionForState(state, { includeMotion: false }); // keep false if you're publishing motion_command directly
      // If you want orchestrator to also drive motion, flip includeMotion:true and remove publishMotion above.
    }

    // Timed transition
    if (state.transition && state.transition.type === 'time') {
      transitionTimeoutRef.current = setTimeout(() => {
        setCurrentStateIndex((prev) => prev + 1);
      }, state.transition.duration);
    }

    return () => clearAllTimeouts();
  }, [ros, currentScenario, currentStateIndex, isPaused, resumeNonce]);

  // Answer click: motion feedback + face feedback + optional orchestrator action
  const handleOptionClick = (selectedOption) => {
    if (!currentScenario) return;
    if (isPaused) return;

    clearAllTimeouts();

    const state = currentScenario.states[currentStateIndex];
    const correct = selectedOption === state.answer;

    // Robot motion feedback
    publishMotion(correct ? 'happy' : 'sad');

    // Face feedback for PyLips
    if (mapleExprRef.current) {
      mapleExprRef.current.publish(new ROSLIB.Message({ data: correct ? 'HAPPY' : 'SAD' }));
    }

    // Optional combined action too (does not need to include motion if you already sent motion_command)
    if (mapleActionRef.current) {
      mapleActionRef.current.publish(
        new ROSLIB.Message({
          data: JSON.stringify({
            expression: correct ? 'HAPPY' : 'SAD',
            face_ms: 1000,
            sync: 'parallel',
          }),
        })
      );
    }

    // Local feedback mp3 (browser)
    setFeedbackAudio(correct ? '/CORRECT.mp3' : '/INCORRECT.mp3');

    feedbackTimeoutRef.current = setTimeout(() => {
      setFeedbackAudio(null);
      if (correct) setCurrentStateIndex((prev) => prev + 1);
    }, 3000);
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
          flexDirection: 'column',
        }}
      >
        {/* Pause/Resume controls */}
        <div style={{ position: 'fixed', top: 16, right: 16, zIndex: 999 }}>
          <button onClick={handlePause} disabled={isPaused} style={{ marginRight: 8 }}>
            Pause
          </button>
          <button onClick={handleResume} disabled={!isPaused}>
            Resume
          </button>
          <div style={{ marginTop: 8, fontSize: 12, opacity: 0.85 }}>
            ROS: {connectionStatus}
          </div>
        </div>

        {currentScenario && currentStateIndex < currentScenario.states.length && (
          <>
            {currentScenario.states[currentStateIndex].image && (
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
            )}

            {currentScenario.states[currentStateIndex].text && (
              <p style={{ fontSize: '1.5em', margin: '20px 0', fontWeight: 'bold' }}>
                {currentScenario.states[currentStateIndex].text}
              </p>
            )}

            {/* Browser-side audio (optional). Robot speech should come from /maple_action tts. */}
            {!isPaused && currentScenario.states[currentStateIndex].audio && (
              <audio
                ref={stateAudioRef}
                key={`state-audio-${currentStateIndex}-${resumeNonce}`}
                src={`/${currentScenario.scenario_name}/audio/${currentScenario.states[currentStateIndex].audio}`}
                autoPlay
              />
            )}

            {currentScenario.states[currentStateIndex].options && (
              <div style={{ marginTop: '20px' }}>
                {currentScenario.states[currentStateIndex].options.map((option, index) => (
                  <button
                    key={index}
                    disabled={isPaused}
                    onClick={() => handleOptionClick(option)}
                    style={{
                      padding: '10px 20px',
                      margin: '10px',
                      fontSize: '1em',
                      cursor: isPaused ? 'not-allowed' : 'pointer',
                      opacity: isPaused ? 0.6 : 1,
                    }}
                  >
                    {option}
                  </button>
                ))}
              </div>
            )}

            {!isPaused && feedbackAudio && (
              <audio ref={feedbackAudioRef} src={feedbackAudio} autoPlay />
            )}

            {isPaused && (
              <div style={{ marginTop: 12, fontSize: 14, opacity: 0.9 }}>
                Paused (Resume replays this section from the start)
              </div>
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
