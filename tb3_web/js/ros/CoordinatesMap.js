//get coordinates_Map data
var map = document.getElementById("coordinates_Map");
var ctx = map.getContext("2d");
//get player data
var view = document.getElementById("player");
var view_button = document.getElementById("Camera-View");

var Camera = document.getElementById("View-or-Map");
function Map_control() {
    if (view.src.slice(-14) == "img/ground.png") {
        console.log("Map Sending");
    }
    else {
        console.log("Don't touch me");
    }
}
function Norm_Angle(angle, T_rad = "true") {
    angle = angle * 180 / Math.PI
    while (angle <= -360 || angle >= 360) {
        if (angle <= -360) {
            angle += 360;
        } else if (angle >= 360) {
            angle -= 360;
        }
    }
    if (T_rad) {
        angle = angle / 180 * Math.PI;
    }
    return angle;
}
function painting(mouse_x, mouse_y,mouse_angle) {
    line_x = mouse_x + 20 * Math.sin(mouse_angle);
    line_y = mouse_y - 20 * Math.cos(mouse_angle);
    ctx.beginPath();
    ctx.lineWidth = 2;
    ctx.strokeStyle = "rgb(55, 0, 255)";
    ctx.arc(mouse_x, mouse_y, 15, 0, 2 * Math.PI);
    ctx.moveTo(mouse_x, mouse_y);
    ctx.lineTo(line_x, line_y);
    ctx.stroke();
    ctx.closePath();
}
function All_paint(){
    for (let i = 0; i < buffer.length; i++){
        painting(buffer[i][0],buffer[i][1],buffer[i][2]);
    }
}

var mouse_clicked = false;
var angle_offset = 0;
var mouse_angle = 0;
var buffer = [];
var coordinate = [];
var current = [];
var button;

map.addEventListener("mousedown", function (e) {
    if (view.src.slice(-14) == "img/ground.png") {
        if (e.button == 0) {
            button = 0;
            mouse_clicked = true;
            mouse_x = e.offsetX;
            mouse_y = e.offsetY;
            ctx.clearRect(0, 0, map.width, map.height);
            All_paint();
            painting(mouse_x, mouse_y,mouse_angle);
            current[0] = mouse_x;
            current[1] = mouse_y;
            current[2] = mouse_angle;
            map.addEventListener("mousemove", function (event) {
                if (mouse_clicked && button == 0) {
                    ctx.clearRect(0, 0, map.width, map.height);
                    All_paint();
                    let move_x = event.offsetX;
                    mouse_angle = Norm_Angle(angle_offset + (mouse_x - move_x) / 180 * Math.PI);
                    painting(mouse_x, mouse_y,mouse_angle);
                    current[2] = mouse_angle;
                }
            })
        } else if (e.button == 2) {
            button = 2;
            mouse_clicked = true;
        } else {
            console.log("Please don't touch the strange button, ok?");
        }

    }
});

map.addEventListener("mouseup", function (e) {
    mouse_clicked = false;
    if (button == 0) {
        console.log(current);
        angle_offset = mouse_angle;
        buffer.push(current);
        current = [];
        console.log(buffer);
    }else if(button ==2){
        buffer.pop();
        console.log(buffer);
        ctx.clearRect(0, 0, map.width, map.height);
        All_paint();
    }
});
map.addEventListener("mouseleave", function (e) {
    mouse_clicked = false;
});
