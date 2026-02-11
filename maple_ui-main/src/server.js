const express = require('express');
const fs = require('fs');
const path = require('path');

const app  = express();
const port = 5000;
const DATA_FILE = path.join(__dirname, 'quiz_metrics.json');

app.use(express.json());

app.post('/api/metrics', (req, res) => {
  const newMetrics = Array.isArray(req.body) ? req.body : [req.body];
  fs.readFile(DATA_FILE, 'utf8', (err, data) => {
    let existing = [];
    if (!err && data) {
      try { existing = JSON.parse(data); } catch (e) { existing = []; }
    }
    const updated = existing.concat(newMetrics);
    fs.writeFile(DATA_FILE, JSON.stringify(updated, null, 2), (err2) => {
      if (err2) return res.status(500).send('Could not write metrics file');
      res.json({ status: 'ok', total: updated.length });
    });
  });
});

app.get('/api/metrics', (req, res) => {
  fs.readFile(DATA_FILE, 'utf8', (err, data) => {
    if (err || !data) return res.json([]);
    try {
      res.json(JSON.parse(data));
    } catch (e) {
      res.json([]);
    }
  });
});

app.delete('/api/metrics', (req, res) => {
  fs.writeFile(DATA_FILE, '[]', err => {
    if (err) return res.status(500).send('Error resetting file');
    res.json({ status: 'reset' });
  });
});

app.listen(port, () => {
  console.log(`Metrics API running on port ${port}`);
});
