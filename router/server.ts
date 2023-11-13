const ws = require("ws");
const roslib = require("roslib");

// no longer needed, i only send geometry, he sends only laserscan.
enum MessageTypes{
    entry,
    geometry,
    laserscan,
    act
};

process.env.NODE_TLS_REJECT_UNAUTHORIZED = "0";

let server:any = undefined;
const agents = new Array();

let setServer = (serverRef):void =>{
    server = serverRef;
}

const laserMessage = {
    "header":{
        "stamp":{
            "secs": 0,
            "nsecs": 0
        },
        "seq": 0,
        "frame_id": ""
    },
    "angle_min": -360 * Math.PI / 180,
    "angle_max": 360 * Math.PI / 180,
    "angle_increment": 0,
    "scan_time": 0,
    "time_increment": 0,
    "range_min": 0.02,
    "range_max": 8,
    "ranges": [0],
    "intensities": [0]
}

let endServer = ():void =>{
    if(server){
        while(agents.length != 0){
            const prevAgent = agents.pop();

            prevAgent.close();
        }

        server.close();

        server = undefined;
    }
}

let startServer = (ros, threshold:number):void =>{
    try{
        setServer(new ws.Server({port:80}));

        console.log("Started the Web Socket Server on Port 80");
        console.log();

        server.on("connection", (agent)=>{
            agent.id = agents.length+1;
            let isGood:boolean = true;
            let namespace:string = `agent${agent.id}/`;
            let vel:string = namespace + "cmd_vel";
            let laser:string = namespace + "raw_laser";
            let laserFrame:string = namespace + "laser";

            // const interval = setInterval(()=>{
            //     if(!isGood){
            //         agent.close();
            //     }

            //     isGood = false;
            // }, 7500);

            if(agent.id > threshold){
                agent.close();
            }
            else{
                console.log(`Agent ${agent.id} Connected`);
                console.log();

                agents.push(agent);

                // const status = new roslib.Topic({
                //     ros: ros,
                //     name: namespace + "status",
                //     messageType: "std_msgs/Bool"
                // });

                const cmdVel = new roslib.Topic({
                    ros: ros,
                    name: vel,
                    messageType: "geometry_msgs/Twist"
                });
            
                const laserScan = new roslib.Topic({
                    ros: ros,
                    name: laser,
                    messageType: "sensor_msgs/LaserScan"
                });

                // status.publish(new roslib.Message({
                //     data: true
                // }));

                cmdVel.subscribe((data)=>{
                    const velMessage = Buffer.alloc(21);

                    velMessage.writeUInt8(MessageTypes.geometry, 0);
                    velMessage.writeFloatLE(data.linear.x, 4);
                    velMessage.writeFloatLE(data.linear.y, 8);
                    velMessage.writeFloatLE(data.angular.z, 12);
                    velMessage.writeUInt32LE(data.linear.z, 16);
            
                    agent.send(velMessage);
                });

                agent.on("message", (data:Buffer)=>{
                    isGood = true;
                    let type:number = data.readUInt8(0);

                    switch(type){
                        case MessageTypes.laserscan:
                            let laserOffset:number = 10;
                            
                            laserMessage.header.seq = data.readUInt32LE(4);
                            laserMessage.header.frame_id = laserFrame;
                            laserMessage.ranges = [];
                            laserMessage.intensities = [];

                            let numPoints:number = data.readUInt16LE(8);

                            laserMessage.angle_increment = 2 * Math.PI  / numPoints;

                            for(let next:number = 0; next < numPoints; ++next){
                                laserMessage.ranges.push(data.readUInt16LE(laserOffset) / 1000);

                                laserOffset+= 2;
                            }

                            for(let next:number = 0; next < numPoints; ++next){
                                laserMessage.intensities.push(data.readUInt8(laserOffset));

                                ++laserOffset;
                            }

                            laserScan.publish(laserMessage);
                    }
                });

                agent.on("close", (code:number)=>{
                    if(agent.id <= threshold){
                        console.log(`Agent ${agent.id} Disconnected with Code ${code}`);
                        console.log();
                        
                        agents.splice(agent.id-1, 1);
    
                        // status.publish(new roslib.Message({
                        //     data: false
                        // }));
                    }
        
                    //clearInterval(interval);
                });
    
                agent.on("error", (error)=>{
                    console.log(error);
                    console.log();
                });
            }
        });
    }
    catch(error){
        console.log(error);
        console.log();
    }
}

export let connectToRos = ():void =>{
    const ros = new roslib.Ros({
        url:"wss://hivebackend1.ddns.net:9090"
    });

    ros.on("connection", ()=>{
        console.log("Connected to the Robotic Backend");

        const numAgents = new roslib.Param({
            ros: ros,
            name: "numAgents",
            //messageType: "std_msgs/Int32"
        });

        numAgents.get((data)=>{
            //let quantity:number = data.data;
            endServer();

            if(data != 0){ //quantity.
                startServer(ros, data);
            }
        });
    });

    ros.on("error", (error)=>{
        console.log(error);
        console.log();
    });

    ros.on("close", ()=>{
        console.log("Disconnected from the Robotic Backend");
        console.log();

        endServer();
    });
}

connectToRos();