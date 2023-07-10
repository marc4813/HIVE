const ws = require("ws");
const roslib = require("roslib");
const https = require("https");
const fs = require("fs");

enum MessageTypes{
    geometry,
    laserscan
}

const ros = new roslib.Ros({
    url: "ws://localhost:9090"
});

//Install wireshark. And apply the parsing changes to the arduino code, as well ssl.

ros.on("connection", ()=>{
    console.log("connected to ros");
});

ros.on("close", ()=>{
    console.log("disconnected from ros");
});

const clients = new Array();

/*const httpsServer = new https.createServer({
    key: fs.readFileSync("key.pem"),
    cert: fs.readFileSync("cert.pem")
});*/

const wss = new ws.Server({port:80});

console.log("server started");

wss.on("connection", (client)=>{
    client.id = clients.length; //adds variance, so if we have 6 agents, and remove 2nd, the replacement agent, will become agent 6.

    clients.push(client);

    console.log(`client ${client.id} connected`);

    const listenerGeometry = new roslib.Topic({
        ros: ros,
        name: "/agent" + client.id+1+"/cmd_vel", //0-based indexing for ids
        messageType: "geometry_msgs/Twist"
    });

    const listenerPresence = new roslib.Topic({ //this is questionable
        ros: ros,
        name: "/presence",
        messageType: "std_msgs/String"
    });

    listenerPresence.publish(new roslib.Message({
        data: client.id,
        active: true
    }));

    listenerGeometry.subscribe((data)=>{
        const message = {
            linear: data.linear,
            angualr: data.angular
        }

        client.send(JSON.stringify(message));
    });

    client.on("message", (data)=>{
        const parsed = JSON.parse(data);

        console.log(parsed);

        switch(parsed.type){
            case MessageTypes.geometry:
                listenerGeometry.publish(new roslib.Message({
                    linear: parsed.linear,
                    angular: parsed.angular
                }));

                break;
        }
    });

    client.on("close", ()=>{
        clients.splice(client.id, 1);

        //Updates client IDs on the side of the router, and will ask the server to do the same.
        for(let i:number = client.id; i < clients.length; i++){
            clients[i].id--;
        }

        listenerPresence.publish(new roslib.Message({
            data: client.id,
            active: false //agent is no longer available.
        }));
    
        
        console.log(`client ${client.id} disconnected`);
    });
});

//httpsServer.listen(8080); //try 443