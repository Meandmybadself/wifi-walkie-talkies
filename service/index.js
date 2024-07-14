const http = require('http');
const WebSocket = require('ws');
const fs = require('fs');

const port = 8080;

// Create an HTTP server
const server = http.createServer((req, res) => {
    if (req.url === '/') {
        fs.readFile('index.html', (err, data) => {
            if (err) {
                res.writeHead(500);
                return res.end('Error loading index.html');
            }
            res.writeHead(200, { 'Content-Type': 'text/html' });
            res.end(data);
        });
    }
});

// Create a WebSocket server
const wss = new WebSocket.Server({ server });

wss.on('connection', function connection(ws) {
    console.log('A new client connected');

    ws.on('message', function incoming(data) {
        if (data instanceof Buffer) {
            console.log('Received binary data of length:', data.length);
            console.log('First 10 bytes:', data.slice(0, 10).toString('hex'));
        } else if (typeof data === 'string') {
            console.log('Received string data:', data);
        } else {
            console.log('Received unknown data type');
        }

        // Forward the data to the browser (ensure this is happening)
        wss.clients.forEach(function each(client) {
            if (client !== ws && client.readyState === WebSocket.OPEN) {
                client.send(data);
            }
        });
    });

    ws.on('close', function close() {
        console.log('A client disconnected');
    });
});

server.listen(port, () => {
    console.log(`Server is running on http://localhost:${port}`);
});