const WebSocket = require("ws");

const server = new WebSocket.Server({port: 80, host: ''});


server.on("connection", (conn) => {
    
    console.log("client connected");

    conn.on("message", (data)=>{
        const packet = JSON.parse(data);

        console.log(packet);

        const response = new Object();

        response.Test = 'test';

        conn.send(response);
    });
});

