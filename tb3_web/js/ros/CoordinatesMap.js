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
function Norm_Angle(angle,T_rad = "true") {
    angle = angle * 180 / Math.PI
    while (angle <= -360 || angle >= 360) {
        if (angle <= -360) {
            angle += 360;
        }else if (angle >= 360) {
            angle -= 360;
        }
    }
    if(T_rad){
        angle = angle / 180 * Math.PI;
    }
    return angle;
}
var mouse_clicked = false;
var angle_offset = 0;
var mouse_angle = 0;
var buffer = [];
var coordinate = [];
var current = [];
map.addEventListener("mousedown", function (e) {
    if (view.src.slice(-14) == "img/ground.png") {
        if (e.button == 0) {
            mouse_clicked = true;
            mouse_x = e.offsetX;
            mouse_y = e.offsetY;
            line_x = mouse_x + 20 * Math.cos(mouse_angle);
            line_y = mouse_y + 20 * Math.sin(mouse_angle);
            // let x_ =(mouse_x);
            // let y_ =(mouse_y);

            // let coord_x = parseInt((mouse_x-x0)/1.28);
            // let coord_y = parseInt((mouse_y-y0)/1.28);

            ctx.beginPath();
            ctx.lineWidth = 2;
            ctx.strokeStyle = "rgb(55, 0, 255)";
            ctx.arc(mouse_x, mouse_y, 10, 0, 2 * Math.PI);
            ctx.moveTo(mouse_x, mouse_y);
            ctx.lineTo(line_x, line_y);
            ctx.stroke();
            ctx.closePath();
            map.addEventListener("mousemove", function (event) {
                if (mouse_clicked) {

                    ctx.clearRect(0, 0, map.width, map.height);
                    let move_x = event.offsetX;
                    mouse_angle = Norm_Angle(angle_offset + (mouse_x - move_x) / 180 * Math.PI);
                    line_x = mouse_x + 20 * Math.cos(mouse_angle);
                    line_y = mouse_y + 20 * Math.sin(mouse_angle);
                    ctx.beginPath();
                    ctx.arc(mouse_x, mouse_y, 15, 0, 2 * Math.PI);
                    ctx.moveTo(mouse_x, mouse_y);
                    ctx.lineTo(line_x, line_y);
                    ctx.stroke();
                }
            })
            current[0] = mouse_x;
            current[1] = mouse_y;
            current[2] = mouse_angle;
            console.log(current);
        } else if (e.button == 2) {
            mouse_clicked = true;
        } else {
            console.log("Please don't touch the strange button, ok?");
        }

    }
});

map.addEventListener("mouseup", function (e) {
    mouse_clicked = false;
    angle_offset = mouse_angle;
    buffer.push(current);
    console.log(buffer);


});
map.addEventListener("mouseleave", function (e) {
    mouse_clicked = false;

});
