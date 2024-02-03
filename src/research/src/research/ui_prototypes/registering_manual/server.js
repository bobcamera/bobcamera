const express = require('express');
const fs = require('fs');
const path = require('path');

const app = express();
const port = 3000;

app.use(express.static(__dirname));

app.get('/directory-listing', (req, res) => {
    const directoryPath = __dirname;

    fs.readdir(directoryPath, (err, files) => {
        if (err) {
            console.error('Error reading directory:', err);
            res.status(500).json({ error: 'Internal Server Error' });
        } else {
            res.json(files);
        }
    });
});

app.get('/', (req, res) => {
    res.sendFile(path.join(__dirname, 'unifiedIndex.html'));
});

app.listen(port, () => {
    console.log(`Server is running at http://localhost:${port}`);
});
