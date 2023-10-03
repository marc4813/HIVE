import { Server } from "http";
import { WebSocketServer } from "ws";

const ws = require("ws");
const roslib = require("roslib");

enum MessageTypes{
    geometry = 1,
    laserscan,
    status
};

export let startServer = ():void =>{
    const agents = new Array();
    let canStart:boolean = false;
    let servRef:WebSocketServer;

    const ros = new roslib.Ros({
        url:"ws://144.126.249.86:9090"
    });

    ros.on("connection", ()=>{
        console.log("SUCCESS: Connected to the Robotic Backend");

        try{
            const server = new ws.Server({port:80});
            servRef = server;

            console.log("SUCCESS: Started the Web Socket Server on Port 80");
    
            server.on("connection", (agent)=>{
                let isActive:boolean = true;
        
                agent.id = agents.length+1;
        
                agents.push(agent);
        
                console.log(`WS: agent ${agent.id} Connected`);
        
                const interval = setInterval(()=>{
                    if(isActive){
                        console.log(true);
                        isActive = false;
        
                        agent.send(JSON.stringify({
                            type: MessageTypes.status
                        }));
                    }
                    else{
                        agent.close();
                    }
                
                }, 5000);
        
                const cmdVel = new roslib.Topic({
                    ros: ros,
                    name: "/agent"+agent.id+"/cmd_vel",
                    messageType: "geometry_msgs/Twist"
                });
        
                const laserScan = new roslib.Topic({
                    ros: ros,
                    name: "/agent"+agent.id+"/laserscan",
                    messageType: "sensor_msgs/Laserscan"
                })
        
                cmdVel.subscribe((data)=>{
                    console.log(`SUBSCRIBED: ${data}`);
        
                    const message = {
                        type: MessageTypes.geometry,
                        linear: data.linear,
                        angular: data.angular
                    }
        
                    agent.send(JSON.stringify(message));
                });
        
                agent.on("message", (data)=>{
                    const parsed = JSON.parse(data);
        
                    console.log(`MESSAGE: ${parsed} from Agent ${agent.id}`);
        
                    switch(parsed.type){
                        case MessageTypes.laserscan: //fix this.
                            let length:number = parsed[10];
                            let ranges:number[] = [];
        
                            for(let next:number = 11; next < length; ++next){
                                ranges.push(parsed[next]);
                            }
        
                            const message = new roslib.Message({
                                header: new roslib.Message({
                                    seq: parsed[0],
                                    time_stamp: parsed[1],
                                    frame_id: parsed[2]
                                }),
        
                                angle_min: parsed[3],
                                angel_max: parsed[4],
                                angle_increment: parsed[5],
                                time_increment: parsed[6],
                                scan_time: parsed[7],
                                range_min: parsed[8],
                                range_max: parsed[9],
                                ranges: ranges
                            });
        
                            laserScan.publish(message);
        
                            console.log(`PUBLISHED: ${message} to Agent ${agent.id}`);
                            //vs publisehd to cmd_vel.
        
                            break;
                        
                        case MessageTypes.status:
                            isActive = true;
                            
                            break;
                            
                        default:
                            console.log("unkown message type"); //remove when deployed.
                    }
                });
        
                agent.on("close", ()=>{
                    console.log(`WS: Agent ${agent.id} Disconnected`);
        
                    clearInterval(interval);
        
                    agents.splice(agent.id-1, 1);
                    
                    for(let i:number = agent.id-1; i < agents.length; i++){
                        agents[i].id--;
                    }
                });
            });
        }
        catch(error){
    
        }
    });

    ros.on("error", (error:string)=>{
        console.log("ERROR: Lost Connection to the Robotic Backend");
    });

    ros.on("close", ()=>{
        if(servRef){ //close through reference.
            servRef.close();
        }
    });
}