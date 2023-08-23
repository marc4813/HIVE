const ws = require("ws");
const roslib = require("roslib");

const MessageTypes = {
    geometry: 1,
    laserscan: 2,
    status: 3
};

const ros = new roslib.Ros({
    url: "ws://0.0.0.0:9090"
});

ros.on("connection", ()=>{
    console.log("connected to ros");
});

ros.on("close", ()=>{
    console.log("disconnected from ros");
});

const clients = new Array();

const server = new ws.Server({port: 80});

console.log("server started");

server.on("connection", (client)=>{
    let isActive:boolean = true;

    client.id = clients.length+1;

    clients.push(client);

    console.log(`client ${client.id} connected`);

    const interval = setInterval(()=>{
        if(isActive){
            console.log(true);
            isActive = false;

            client.send(JSON.stringify({
                type: MessageTypes.status
            }));
        }
        else{
            client.close();
        }
        
    }, 5000);

    const incomingGeometry = new roslib.Topic({
        ros: ros,
        name: "/agent"+client.id+"/in_cmd_vel",
        messageType: "geometry_msgs/Twist"
    });

    const outgoingGeometry = new roslib.Topic({
        ros: ros,
        name: "/agent"+client.id+"/out_cmd_vel",
        messageType: "std_msgs/String"
    });

    incomingGeometry.subscribe((data)=>{
        const message = {
            type: MessageTypes.geometry,
            linear: data.linear,
            angular: data.angular
        }

        client.send(JSON.stringify(message));
    });

    client.on("message", (data)=>{
        const parsed = JSON.parse(data);

        console.log(parsed);

        const message = new roslib.Message({
            data: JSON.stringify({
                agent: client.id,
                status: parsed.data
            })
        });

        switch(parsed.type){
            case MessageTypes.geometry:
                outgoingGeometry.publish(message);
                break;

            case MessageTypes.laserscan:
                break;
            
            case MessageTypes.status:
                isActive = true;
                break;
                
            default:
                console.log("unkown message type");
        }
    });

    client.on("close", ()=>{
        console.log(`client ${client.id} disconnected`);

        clearInterval(interval);

        clients.splice(client.id-1, 1);
        
        for(let i:number = client.id-1; i < clients.length; i++){
            clients[i].id--;
        }
    });
});