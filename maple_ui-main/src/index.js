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

  // Story lifecycle
  const [hasStarted, setHasStarted] = useState(false);

  // Pause/play state
  // Start paused so the browser does not try to autoplay audio before user interaction.
  const [isPaused, setIsPaused] = useState(true);
  const [resumeNonce, setResumeNonce] = useState(0);

  // Timers and audio refs (browser audio)
  const transitionTimeoutRef = useRef(null);
  const feedbackTimeoutRef = useRef(null);
  const stateAudioRef = useRef(null);
  const feedbackAudioRef = useRef(null);

  // ROS topics (refs)
  const motionTopicRef = useRef(null);              // /motion_command
  const interactionControlRef = useRef(null);       // /interaction_control
  const mapleActionRef = useRef(null);              // /maple_action
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

  const publishMapleActionForState = (state, { includeMotion = true } = {}) => {
    if (state.expression && mapleExprRef.current) {
      mapleExprRef.current.publish(new ROSLIB.Message({ data: String(state.expression) }));
    }
    if (state.appearance && mapleAppearanceRef.current) {
      mapleAppearanceRef.current.publish(new ROSLIB.Message({ data: String(state.appearance) }));
    }

    const payload = {};
    if (includeMotion && state.motion) payload.motion = state.motion;
    if (state.tts) payload.tts = state.tts;

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

  const handleStart = () => {
    if (hasStarted) return;
    setHasStarted(true);
    setIsPaused(false);

    // If your ROS side only understands pause/resume, resume is safe here.
    publishInteractionControl('resume');

    // Also re-trigger speech/face from the start of the first state (no motion here)
    if (currentScenario && currentStateIndex < currentScenario.states.length) {
      const state = currentScenario.states[currentStateIndex];
      publishMapleActionForState(state, { includeMotion: false });
    }

    // Kick the audio effect (helps if audio element is already mounted quickly)
    setResumeNonce((n) => n + 1);
  };

  const handlePause = () => {
    if (!hasStarted) return;
    if (isPaused) return;
    setIsPaused(true);

    clearAllTimeouts();
    stopAllBrowserAudio();
    setFeedbackAudio(null);

    publishInteractionControl('pause');
  };

  // “Play” is just UI naming. Robot controller still expects the word "resume".
  const handlePlay = () => {
    if (!hasStarted) return;
    if (!isPaused) return;
    setIsPaused(false);

    publishInteractionControl('resume');

    // Restart browser audio + timers for this state
    setResumeNonce((n) => n + 1);

    // Re-trigger speech/face from start (do not re-trigger motion here)
    if (currentScenario && currentStateIndex < currentScenario.states.length) {
      const state = currentScenario.states[currentStateIndex];
      publishMapleActionForState(state, { includeMotion: false });
    }

    // If there's an audio element for the current state, play it explicitly.
    if (stateAudioRef.current) {
      stateAudioRef.current.currentTime = 0;
      stateAudioRef.current.play().catch((error) => {
        console.warn('Audio playback prevented by browser:', error);
      });
    }
  };

  // ROS connection
  useEffect(() => {
    const rosInstance = new ROSLIB.Ros({
      url: 'ws://192.168.10.101:9090',
      // or: url: `ws://${window.location.hostname}:9090`,
    });

    rosInstance.on('connection', () => {
      console.log('Connected to ROS');
      setConnectionStatus('Connected');
      setRos(rosInstance);

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
    fetch('/classroom_interaction/scenario_config_main.json')
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
    if (!hasStarted) return;
    if (isPaused) return;
    if (currentStateIndex >= currentScenario.states.length) return;

    const state = currentScenario.states[currentStateIndex];

    const key = `state-${currentStateIndex}-resume-${resumeNonce}`;
    if (lastPublishKeyRef.current !== key) {
      lastPublishKeyRef.current = key;

      if (state.motion) publishMotion(state.motion);

      // speech/face
      publishMapleActionForState(state, { includeMotion: false });
    }

    if (state.transition && state.transition.type === 'time') {
      transitionTimeoutRef.current = setTimeout(() => {
        setCurrentStateIndex((prev) => prev + 1);
      }, state.transition.duration);
    }

    return () => clearAllTimeouts();
  }, [ros, currentScenario, currentStateIndex, isPaused, resumeNonce, hasStarted]);

  // Try to play the current state's audio whenever it changes or when play is resumed.
  useEffect(() => {
    if (!hasStarted) return;
    if (isPaused) return;
    if (!currentScenario) return;
    if (currentStateIndex >= (currentScenario?.states?.length ?? 0)) return;

    const state = currentScenario.states[currentStateIndex];
    if (state.audio && stateAudioRef.current) {
      stateAudioRef.current.currentTime = 0;
      const playPromise = stateAudioRef.current.play();
      if (playPromise !== undefined) {
        playPromise.catch((error) => {
          console.warn('Audio playback prevented by browser:', error);
        });
      }
    }
  }, [currentStateIndex, resumeNonce, isPaused, currentScenario, hasStarted]);

  const handleOptionClick = (selectedOption) => {
    if (!currentScenario || isPaused) return;
    clearAllTimeouts();
  
    const state   = currentScenario.states[currentStateIndex];
    const correct = selectedOption === state.answer;
    // choose lowercase for motion and expression names
    const expressionName = correct ? 'happy' : 'sad';
  
    // publish the motion
    publishMotion(expressionName);
  
    // publish the expression on /maple_expression
    if (mapleExprRef.current) {
      mapleExprRef.current.publish(new ROSLIB.Message({ data: expressionName }));
    }
  
    // publish the action on /maple_action
    if (mapleActionRef.current) {
      mapleActionRef.current.publish(
        new ROSLIB.Message({
          data: JSON.stringify({
            expression: expressionName,
            face_ms: 1000,
            sync: 'parallel',
          }),
        })
      );
    }
  
    // play feedback sound and advance state when correct
    setFeedbackAudio(correct ? '/CORRECT.mp3' : '/INCORRECT.mp3');
    feedbackTimeoutRef.current = setTimeout(() => {
      setFeedbackAudio(null);
      if (correct) setCurrentStateIndex((prev) => prev + 1);
    }, 3000);
  };

  const currentState =
    currentScenario && currentStateIndex < currentScenario.states.length
      ? currentScenario.states[currentStateIndex]
      : null;

  const ControlsBelowImage = () => (
    <div
      style={{
        display: 'flex',
        gap: 10,
        alignItems: 'center',
        justifyContent: 'center',
        marginTop: 6,
        marginBottom: 10,
      }}
    >
      {!hasStarted ? (
        <button onClick={handleStart} disabled={!currentScenario}>
          Start
        </button>
      ) : (
        <>
          {!isPaused ? (
            <button onClick={handlePause}>Pause</button>
          ) : (
            <button onClick={handlePlay}>Play</button>
          )}
        </>
      )}

      <span style={{ fontSize: 12, opacity: 0.8 }}>
        ROS: {connectionStatus}
      </span>
    </div>
  );

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
        {currentState && (
          <>
            {currentState.image && (
              <img
                src={`/${currentScenario.scenario_name}/img/${currentState.image}`}
                alt="state"
                style={{
                  maxWidth: '100%',
                  maxHeight: '60vh',
                  margin: '20px',
                  objectFit: 'contain',
                }}
              />
            )}

            {/* Controls directly below the image */}
            <ControlsBelowImage />

            {currentState.text && (
              <p style={{ fontSize: '1.5em', margin: '10px 0', fontWeight: 'bold' }}>
                {currentState.text}
              </p>
            )}

            {/* Render the audio element after start; playback is triggered via play() in effects/click handlers */}
            {hasStarted && currentState.audio && (
              <audio
                ref={stateAudioRef}
                key={`state-audio-${currentStateIndex}-${resumeNonce}`}
                src={`/${currentScenario.scenario_name}/audio/${currentState.audio}`}
                preload="auto"
              />
            )}

            {currentState.options && (
              <div style={{ marginTop: '20px' }}>
                {currentState.options.map((option, index) => (
                  <button
                    key={index}
                    disabled={!hasStarted || isPaused}
                    onClick={() => handleOptionClick(option)}
                    style={{
                      padding: '10px 20px',
                      margin: '10px',
                      fontSize: '1em',
                      cursor: !hasStarted || isPaused ? 'not-allowed' : 'pointer',
                      opacity: !hasStarted || isPaused ? 0.6 : 1,
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

            {hasStarted && isPaused && (
              <div style={{ marginTop: 12, fontSize: 14, opacity: 0.9 }}>
                Paused (Play replays this section from the start)
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
