import React, { useEffect, useRef, useState } from 'react';
import ReactDOM from 'react-dom/client';
import ROSLIB from 'roslib';

/*
  This UI does four big things:

  1) Connects to ROS (rosbridge websocket) so we can publish robot actions.
  2) Loads a scenario JSON and steps through its states.
  3) Plays audio and shows images and quiz options from the scenario.
  4) Records quiz interaction metrics:
     - which option was pressed
     - whether it was correct
     - how long it took to respond
     Metrics are hidden on the end screen until the researcher presses Ctrl+M.
*/

const root = ReactDOM.createRoot(document.getElementById('root'));

function App() {
  /* -----------------------------
     React state: UI and story state
  ------------------------------ */

  // ROS connection status text
  const [connectionStatus, setConnectionStatus] = useState('Disconnected');

  // ROS instance object (from roslib)
  const [ros, setRos] = useState(null);

  // Loaded scenario JSON
  const [currentScenario, setCurrentScenario] = useState(null);

  // Which scenario "state" we are currently showing
  const [currentStateIndex, setCurrentStateIndex] = useState(0);

  // Feedback audio file for correct or incorrect responses
  const [feedbackAudio, setFeedbackAudio] = useState(null);

  // Has the participant pressed Start yet?
  const [hasStarted, setHasStarted] = useState(false);

  // Pause flag: true means we do not advance, no audio continues
  const [isPaused, setIsPaused] = useState(true);

  // Increment to force replay of state audio on resume
  const [resumeNonce, setResumeNonce] = useState(0);

  // End screen flag when scenario finishes
  const [showEndScreen, setShowEndScreen] = useState(false);

  // Metrics table visibility on end screen (researcher toggles via Ctrl+M)
  const [metricsVisible, setMetricsVisible] = useState(false);

  // Metrics loaded from backend via GET /api/metrics
  const [serverMetrics, setServerMetrics] = useState([]);

  /* -----------------------------
     Refs: values that should persist without re-render
  ------------------------------ */

  // Timeouts for state transitions and feedback delays
  const transitionTimeoutRef = useRef(null);
  const feedbackTimeoutRef = useRef(null);

  // Audio elements in the browser
  const stateAudioRef = useRef(null);
  const feedbackAudioRef = useRef(null);

  // ROS publishers (topics). Stored in refs so we do not recreate them constantly.
  const motionTopicRef = useRef(null);
  const interactionControlRef = useRef(null);
  const mapleActionRef = useRef(null);
  const mapleExprRef = useRef(null);
  const mapleAppearanceRef = useRef(null);

  // Used to prevent duplicate publishes due to React StrictMode in development
  const lastPublishKeyRef = useRef('');

  // For measuring response time on quiz states
  // Set when the quiz options appear, cleared after the user answers
  const questionStartTimeRef = useRef(null);

  // All local metrics recorded this run (not automatically saved unless you call save)
  const metricsRef = useRef([]);

  /* -----------------------------
     Helper functions: timers and audio
  ------------------------------ */

  // Cancel any scheduled timeouts
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

  // Stop and rewind any browser audio
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

  /* -----------------------------
     Helper functions: ROS publishing
  ------------------------------ */

  // Publish a command on /interaction_control (resume or pause)
  const publishInteractionControl = (cmd) => {
    const t = interactionControlRef.current;
    if (!t) return;
    t.publish(new ROSLIB.Message({ data: cmd }));
  };

  // Publish a motion command on /motion_command
  const publishMotion = (motionName) => {
    const t = motionTopicRef.current;
    if (!t || !motionName) return;
    t.publish(new ROSLIB.Message({ data: motionName }));
  };

  /*
    publishMapleActionForState:

    Many states may have fields like:
      - motion
      - tts
      - expression
      - appearance
      - face_ms
      - sync
      - wait_speech

    This function constructs a JSON payload for /maple_action
    and optionally publishes expression and appearance on their own topics.
  */
  const publishMapleActionForState = (state, { includeMotion = true } = {}) => {
    // Publish expression as a separate topic if present
    if (state.expression && mapleExprRef.current) {
      mapleExprRef.current.publish(new ROSLIB.Message({ data: String(state.expression) }));
    }

    // Publish appearance as a separate topic if present
    if (state.appearance && mapleAppearanceRef.current) {
      mapleAppearanceRef.current.publish(new ROSLIB.Message({ data: String(state.appearance) }));
    }

    // Build /maple_action payload
    const payload = {};
    if (includeMotion && state.motion) payload.motion = state.motion;
    if (state.tts) payload.tts = state.tts;
    if (state.expression) payload.expression = state.expression;
    if (state.appearance) payload.appearance = state.appearance;
    if (state.face_ms) payload.face_ms = state.face_ms;
    if (state.sync) payload.sync = state.sync;
    if (state.wait_speech !== undefined) payload.wait_speech = !!state.wait_speech;

    // Publish only if we have at least one key
    if (mapleActionRef.current && Object.keys(payload).length > 0) {
      mapleActionRef.current.publish(new ROSLIB.Message({ data: JSON.stringify(payload) }));
      console.log('Published /maple_action', payload);
    }
  };

  /* -----------------------------
     UI controls: start, pause, play
  ------------------------------ */

  // Start begins the scenario, unpauses, and triggers the first state
  const handleStart = () => {
    if (hasStarted) return;

    setHasStarted(true);
    setIsPaused(false);

    // Tell robot side to resume
    publishInteractionControl('resume');

    // Trigger first state action (often face/tts) without motion replay
    if (currentScenario && currentStateIndex < currentScenario.states.length) {
      const state = currentScenario.states[currentStateIndex];
      publishMapleActionForState(state, { includeMotion: false });
    }

    // Used to force audio replay on some browsers
    setResumeNonce((n) => n + 1);
  };

  // Pause stops timers and audio, and tells ROS to pause
  const handlePause = () => {
    if (!hasStarted) return;
    if (isPaused) return;

    setIsPaused(true);
    clearAllTimeouts();
    stopAllBrowserAudio();
    setFeedbackAudio(null);
    publishInteractionControl('pause');
  };

  // Play resumes from pause, replaying the current state from the start
  const handlePlay = () => {
    if (!hasStarted) return;
    if (!isPaused) return;

    setIsPaused(false);
    publishInteractionControl('resume');

    // Force audio effect to rerun
    setResumeNonce((n) => n + 1);

    // Retrigger state action (tts/face) without repeating motion
    if (currentScenario && currentStateIndex < currentScenario.states.length) {
      const state = currentScenario.states[currentStateIndex];
      publishMapleActionForState(state, { includeMotion: false });
    }

    // Try to replay state audio
    if (stateAudioRef.current) {
      stateAudioRef.current.currentTime = 0;
      stateAudioRef.current.play().catch((error) => {
        console.warn('Audio playback prevented by browser:', error);
      });
    }
  };

  /* -----------------------------
     Effect: connect to ROS via rosbridge websocket
  ------------------------------ */

  useEffect(() => {
    // Update this URL for your network as needed
    const rosInstance = new ROSLIB.Ros({ url: 'ws://192.168.10.101:9090' });

    rosInstance.on('connection', () => {
      console.log('Connected to ROS');
      setConnectionStatus('Connected');
      setRos(rosInstance);

      // Set up topics (publishers)
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

      // Clear refs so we do not publish to dead topics
      motionTopicRef.current = null;
      interactionControlRef.current = null;
      mapleActionRef.current = null;
      mapleExprRef.current = null;
      mapleAppearanceRef.current = null;
    });

    // Cleanup: close ROS connection on unmount
    return () => {
      try {
        rosInstance.close();
      } catch (_) {}
    };
  }, []);

  /* -----------------------------
     Effect: load scenario JSON from public folder
  ------------------------------ */

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

  /* -----------------------------
     Effect: on state entry
     - publish robot actions
     - schedule time-based transitions
     - start quiz timer if state has options
  ------------------------------ */

  useEffect(() => {
    clearAllTimeouts();

    // Do not run if not ready
    if (!ros || !currentScenario) return;
    if (!hasStarted) return;
    if (isPaused) return;

    // Finished scenario: do not run state logic
    if (currentStateIndex >= currentScenario.states.length) return;

    const state = currentScenario.states[currentStateIndex];

    // Avoid duplicate publishes from StrictMode double-invocation
    const key = `state-${currentStateIndex}-resume-${resumeNonce}`;
    if (lastPublishKeyRef.current !== key) {
      lastPublishKeyRef.current = key;

      // If the state defines a motion, publish it
      if (state.motion) publishMotion(state.motion);

      // Publish TTS, face, appearance, etc (without repeating motion)
      publishMapleActionForState(state, { includeMotion: false });
    }

    // If this state is a quiz, set the timer start time
    if (state.options) {
      questionStartTimeRef.current = performance.now();
    } else {
      questionStartTimeRef.current = null;
    }

    // If the state transitions automatically after a duration, schedule it
    if (state.transition && state.transition.type === 'time') {
      transitionTimeoutRef.current = setTimeout(() => {
        setCurrentStateIndex((prev) => prev + 1);
      }, state.transition.duration);
    }

    return () => clearAllTimeouts();
  }, [ros, currentScenario, currentStateIndex, isPaused, resumeNonce, hasStarted]);

  /* -----------------------------
     Effect: play state audio when entering a state (and when resuming)
  ------------------------------ */

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

  /* -----------------------------
     Effect: detect end of scenario and show end screen
  ------------------------------ */

  useEffect(() => {
    if (
      currentScenario &&
      hasStarted &&
      currentStateIndex >= currentScenario.states.length &&
      !showEndScreen
    ) {
      setShowEndScreen(true);
    }
  }, [currentScenario, currentStateIndex, hasStarted, showEndScreen]);

  /* -----------------------------
     Effect: when end screen appears, hide metrics by default
  ------------------------------ */

  useEffect(() => {
    if (showEndScreen) setMetricsVisible(false);
  }, [showEndScreen]);

  /* -----------------------------
     Effect: keyboard shortcut on end screen (Ctrl+M)
     toggles showing the metrics
  ------------------------------ */

  useEffect(() => {
    if (!showEndScreen) return;

    const onKeyDown = (e) => {
      const isCtrlM = e.ctrlKey && (e.key === 'm' || e.key === 'M');
      if (!isCtrlM) return;

      e.preventDefault();
      setMetricsVisible((v) => !v);
    };

    window.addEventListener('keydown', onKeyDown);
    return () => window.removeEventListener('keydown', onKeyDown);
  }, [showEndScreen]);

  /* -----------------------------
     Backend calls:
     - save local metrics array to backend
     - load previously saved metrics from backend
  ------------------------------ */

  const saveMetricsToBackend = async () => {
    try {
      const response = await fetch('/api/metrics', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(metricsRef.current),
      });
      if (!response.ok) throw new Error('Failed to save metrics');
      alert('Metrics saved');
    } catch (err) {
      console.error(err);
    }
  };

  const loadMetricsFromBackend = async () => {
    try {
      const response = await fetch('/api/metrics');
      if (!response.ok) throw new Error('Failed to fetch metrics');
      const data = await response.json();
      setServerMetrics(data);
    } catch (err) {
      console.error(err);
    }
  };

  /* -----------------------------
     Quiz interaction:
     records a metric, publishes robot feedback, and advances if correct
  ------------------------------ */

  const handleOptionClick = (selectedOption) => {
    if (!currentScenario || isPaused) return;

    // Stop any pending timers before handling click
    clearAllTimeouts();

    const state = currentScenario.states[currentStateIndex];

    // Determine correctness
    const correct = selectedOption === state.answer;

    // Motion/expression name for feedback
    const expressionName = correct ? 'happy' : 'sad';

    // Compute response time if we have a quiz timer started
    const now = performance.now();
    let timeToPress = null;
    if (questionStartTimeRef.current !== null) {
      timeToPress = now - questionStartTimeRef.current;
    }

    // Store locally
    metricsRef.current.push({
      stateIndex: currentStateIndex,
      selectedOption,
      correct,
      timeToPress,
    });

    // Clear timer for this question after answer
    questionStartTimeRef.current = null;

    // Publish robot feedback
    publishMotion(expressionName);

    if (mapleExprRef.current) {
      mapleExprRef.current.publish(new ROSLIB.Message({ data: expressionName }));
    }

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

    // Play browser feedback sound and after 3 seconds advance if correct
    setFeedbackAudio(correct ? '/CORRECT.mp3' : '/INCORRECT.mp3');
    feedbackTimeoutRef.current = setTimeout(() => {
      setFeedbackAudio(null);
      if (correct) setCurrentStateIndex((prev) => prev + 1);
    }, 3000);
  };

  /* -----------------------------
     Small component: renders a table of metrics
  ------------------------------ */

  const MetricsTable = ({ metrics }) => {
    if (!metrics || metrics.length === 0) return null;

    return (
      <table style={{ marginTop: '20px', borderCollapse: 'collapse' }}>
        <thead>
          <tr>
            <th style={{ border: '1px solid #ccc', padding: '4px' }}>Question #</th>
            <th style={{ border: '1px solid #ccc', padding: '4px' }}>Selected Option</th>
            <th style={{ border: '1px solid #ccc', padding: '4px' }}>Correct?</th>
            <th style={{ border: '1px solid #ccc', padding: '4px' }}>Time to Press (ms)</th>
          </tr>
        </thead>
        <tbody>
          {metrics.map((m, idx) => (
            <tr key={idx}>
              <td style={{ border: '1px solid #ccc', padding: '4px', textAlign: 'center' }}>
                {m.stateIndex}
              </td>
              <td style={{ border: '1px solid #ccc', padding: '4px', textAlign: 'center' }}>
                {m.selectedOption}
              </td>
              <td style={{ border: '1px solid #ccc', padding: '4px', textAlign: 'center' }}>
                {m.correct ? 'Yes' : 'No'}
              </td>
              <td style={{ border: '1px solid #ccc', padding: '4px', textAlign: 'center' }}>
                {m.timeToPress != null ? Math.round(m.timeToPress) : 'N/A'}
              </td>
            </tr>
          ))}
        </tbody>
      </table>
    );
  };

  /* -----------------------------
     Current state helper
  ------------------------------ */

  const currentState =
    currentScenario && currentStateIndex < currentScenario.states.length
      ? currentScenario.states[currentStateIndex]
      : null;

  /* -----------------------------
     End screen UI:
     Metrics hidden until Ctrl+M
  ------------------------------ */

  if (showEndScreen) {
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
          <h2>Session Complete</h2>
          <p>Thank you.</p>

          <p style={{ marginTop: '12px', fontSize: '0.95em', opacity: 0.7 }}>
            Researcher: press Ctrl+M to toggle metrics
          </p>

          {metricsVisible && (
            <>
              <p style={{ marginTop: '18px' }}>Below are the interaction metrics for this session.</p>

              <MetricsTable metrics={metricsRef.current} />

              <div style={{ marginTop: '20px' }}>
                <button onClick={saveMetricsToBackend} style={{ marginRight: '10px' }}>
                  Save metrics
                </button>
                <button onClick={loadMetricsFromBackend}>Load saved metrics</button>
              </div>

              {serverMetrics && serverMetrics.length > 0 && (
                <div style={{ marginTop: '20px', textAlign: 'left' }}>
                  <h3>Metrics retrieved from backend</h3>
                  <MetricsTable metrics={serverMetrics} />
                </div>
              )}
            </>
          )}
        </header>
      </div>
    );
  }

  /* -----------------------------
     Main story UI:
     shows image, text, quiz options, and audio
  ------------------------------ */

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
            {/* State image (if present) */}
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

            {/* Controls row */}
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
              {/* Start button only shows before session begins */}
              {!hasStarted ? (
                <button
                  onClick={handleStart}
                  disabled={!currentScenario}
                  style={{
                    padding: '14px 28px',
                    fontSize: '1.25em',
                    borderRadius: '10px',
                    cursor: !currentScenario ? 'not-allowed' : 'pointer',
                    opacity: !currentScenario ? 0.6 : 1,
                  }}
                >
                  Start
                </button>
              ) : (
                <>
                  {/* Pause or Play depending on current pause state */}
                  {!isPaused ? (
                    <button onClick={handlePause} style={{ padding: '10px 18px', fontSize: '1em' }}>
                      Pause
                    </button>
                  ) : (
                    <button onClick={handlePlay} style={{ padding: '10px 18px', fontSize: '1em' }}>
                      Play
                    </button>
                  )}
                </>
              )}

              {/* ROS connection status */}
              <span style={{ fontSize: 12, opacity: 0.8 }}>ROS: {connectionStatus}</span>
            </div>

            {/* State text (if present) */}
            {currentState.text && (
              <p style={{ fontSize: '1.5em', margin: '10px 0', fontWeight: 'bold' }}>
                {currentState.text}
              </p>
            )}

            {/* State audio element (if present) */}
            {hasStarted && currentState.audio && (
              <audio
                ref={stateAudioRef}
                key={`state-audio-${currentStateIndex}-${resumeNonce}`}
                src={`/${currentScenario.scenario_name}/audio/${currentState.audio}`}
                preload="auto"
              />
            )}

            {/* Quiz buttons (if state defines options) */}
            {currentState.options && (
              <div style={{ marginTop: '20px' }}>
                {currentState.options.map((option, index) => (
                  <button
                    key={index}
                    disabled={!hasStarted || isPaused}
                    onClick={() => handleOptionClick(option)}
                    style={{
                      width: '150px', // Match the start button width
                      height: '50px', // Match the start button height
                      padding: '10px 20px',
                      borderRadius: '10px',
                      //fill color in boxes for each option
                        backgroundColor: '#FDBA90',
                      margin: '10px',
                      fontSize: '1em',
                      cursor: !hasStarted || isPaused ? 'not-allowed' : 'pointer',
                      opacity: !hasStarted || isPaused ? 0.6 : 1,
                      textAlign: 'center', // Ensure text is centered
                    }}
                  >
                    {option}
                  </button>
                ))}
              </div>
            )}

            {/* Feedback audio plays only if not paused */}
            {!isPaused && feedbackAudio && <audio ref={feedbackAudioRef} src={feedbackAudio} autoPlay />}

            {/* Pause hint */}
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
