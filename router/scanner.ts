import {Token} from "./router";
//const {test } = require("./router") check if it time left.

export class Scanner{
    private static getType = (sym:string):string =>{
        let ascii:number = sym.charCodeAt(0);
        let res:string = "";

        if((ascii >= 65 && ascii <= 90) || (ascii >= 97 && ascii <= 122)){
            res = "alpha";
        }
        else if(ascii >= 48 && ascii <= 57){
            res = "digit";
        }
        else if(ascii >= 0 && ascii <=31){
            res = "cntrl";
        }
        else if(ascii == 32){
            res = "space"
        }
        else if(ascii == 10){
            res = "newline";
        }
        else{
            if((ascii >= 33 && ascii <= 47) || (ascii >= 58 && ascii <= 64)){
                res = "punct";
            }
            else{
                if((ascii >= 91 && ascii <= 96) || (ascii >= 123 && ascii <= 126)){
                    res = "punct";
                }
            }
        }

        return res;
    }

    public static getOsTokens = (data:string[]):Token[] =>{
        let tokens:Token[] = [];
        let buffer:string = "";
        data.push('\n');

        for(let i:number = 0; i < data.length; ++i){
            let sym:string = data[i];
            let type:string = this.getType(sym);
            
            if(type == "alpha" || type == "digit" || type == "space"){
                buffer+=sym;
            }
            else if(type == "punct"){
                if(sym == '/' || sym == '.' || sym == '_' || sym == ':' || sym == '(' || sym == ')'){
                    buffer+=sym;
                }
                else if(sym == '"'){
                    continue;
                }
                else{
                    if(sym == '='){
                        if(buffer == "NAME" || buffer == "VERSION_CODENAME"){
                            tokens.push(new Token(buffer));
                        }
                    }

                    buffer = "";
                }
            }
            else{
                // Carriage return before newline.
                if(tokens.length > 0 && !tokens[tokens.length-1].data){
                    tokens[tokens.length-1].data = buffer;
                }

                buffer = "";
            }
        }

        return tokens;
    }

    public static getFreqTokens = (data:string[]):Token[] =>{
        let tokens:Token[] = [];
        let buffer:string = "";

        for(let symIndex:number = 0; symIndex < data.length; ++symIndex){
            let sym:string = data[symIndex];
            let type:string = this.getType(sym);

            if(type == "alpha" || type == "digit"){
                if(buffer == "wlan" || buffer == "uap"){
                    tokens.push(new Token(buffer));

                    buffer = "";
                }

                buffer+= sym;
            }
            else{
                if(tokens.length > 0){
                    let prevToken:Token = tokens[tokens.length-1];

                    if(prevToken.id == -1){
                        tokens[tokens.length-1].id = Number(buffer);
                    }
                    else{
                        if(sym == '('){
                            tokens[tokens.length-1].data = 'C';
                        }
                        else{
                            if(prevToken.freq == -1 && prevToken.data == 'C'){
                                let tempNumber:string = buffer[0] + '.' + buffer[1];
                                tokens[tokens.length-1].freq = Number(tempNumber);
                                tokens[tokens.length-1].data = "";
                            }
                        }
                    }
                }

                buffer = "";
            }
        }

        return tokens;
    }

    public static getNetTokens = (data:string[]):Token[] =>{
        let tokens:Token[] = [];
        let buffer:string = "";
        
        for(let symIndex:number = 0; symIndex < data.length; ++symIndex){
            let sym:string = data[symIndex];
            let type:string = this.getType(sym);

            if(type == "alpha" || type == "digit" || sym == '.'){
                if(type == "digit"){
                    if(buffer == "wlan" || buffer == "uap"){
                        tokens.push(new Token(buffer));

                        buffer = "";
                    }
                }

                buffer+=sym;
            }
            else if(type == "space"){
                //console.log(tokens[tokens.length-1].data == "I");
                if(tokens.length > 0){
                    if(!tokens[tokens.length-1].data || tokens[tokens.length-1].data == "I"){
                        if(buffer == "inet"){
                            tokens[tokens.length-1].data = "I";
                        }
                        else{
                            if(tokens[tokens.length-1].data == "I"){
                                if(buffer == "127.0.0.1"){
                                    tokens[tokens.length-1].data = "";
                                }
                                else{
                                    tokens[tokens.length-1].data = buffer;
                                }
                            }
                        }
                    }
                }

                buffer = "";
            }
            else{
                if(sym == ':'){
                    if(tokens.length > 0 && tokens[tokens.length-1].id < 0){
                        tokens[tokens.length-1].id = Number(buffer);
                    }
                }

                buffer = "";
            }
        }

        return tokens;
    }
}