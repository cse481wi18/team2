/**
 * Created by Case-Sensitive on 3/9/2018.
 */

let setStatus1 = function(text, text2) {
    $("#status1").html(text);
    setStatus2("");
};

let setStatus2 = function(text) {
    $("#status2").html(text);
};

// Map functions below
a = function() {
    let map = new map_class(".map_canvas");

    setInterval(() => { map.update() }, 300);
    return map;
};

let map_class = function(selector) {

    this.meterSize = 100;
    this.selector = selector;

    // Marks: {x: ..., y: ...}
    this.orangeMarks = [];
    this.blueMarks = [];
    this.greenMarks = [];
    this.greenCircleMarks = [];
    this.robotPoint = {x: 0, y: 0};
    this.robotAngle = 0;

    this.orangeMarkElements = [];
    this.greenMarkElements = [];
    this.greenCircleMarkElements = [];
    this.blueMarkElements = [];
    this.blackMarkElements = [];
};
map_class.prototype.setOrangeMarks = function(marks) {
    this.orangeMarks = marks;
};
map_class.prototype.setBlueMarks = function(marks) {
    this.blueMarks = marks;
};
map_class.prototype.setGreenCircleMarks = function(marks) {
    this.greenCircleMarks = marks;
};
map_class.prototype.setMeterSize = function(meterSize) {
    this.meterSize = meterSize;
};
map_class.prototype.setRobotPose = function(p2d) {
    this.robotPoint = p2d;
}
map_class.prototype.setRobotAngle = function(angle) {
    this.robotAngle = angle;
};
map_class.prototype.updateMarks = function(marks, markElements, imgSrc, imgWidth, imgHeight, angle) {
    // var b = Polymer.dom(this.root).querySelector("div");
    
    let mapElement = $(this.selector);
    // mapElement.css("background-color", "cyan");
    // console.log("hello");
    // console.log(mapElement);
    // mapElement.empty();
    let newElements = [];
    for (let i = 0; i < marks.length; i++) {
        let mark = marks[i];
        let imgElement;
        let styles = {};
        let positionStyles = this.getPositionStyles(mark.x, mark.y, imgWidth, imgHeight);
        styles = $.extend(styles, positionStyles);
        // console.log(styles);
        if (i < markElements.length) {
            imgElement = markElements[i];
        } else {
            imgElement = $(document.createElement("img"))
                .addClass("ball_marker")
                .attr("src", imgSrc)
                .css("opacity", 0.0)
                .css("width", imgWidth + "px")
                .css("height", imgHeight + "px")
                .css(positionStyles);
            console.log("A: " + imgElement.length);
            styles = $.extend(styles, {opacity: 1.0});
            
            mapElement.append(imgElement);
        }
        if (angle) {
            let lastAngle;
            if (imgElement._lastAngle === undefined) {
                lastAngle = 0;
            } else {
                lastAngle = imgElement._lastAngle;
            }
            imgElement.animateRotate(lastAngle, angle, 80);
            imgElement._lastAngle = angle;
        }
        // imgElement.css(styles);
        imgElement.animate(styles, 200);
        newElements.push(imgElement);
    }
    for (let i = marks.length; i < markElements.length; i++) {
        let imgElement = markElements[i];
        let styles = {opacity: 0.0};
        // imgElement.css(styles);
        imgElement.animate(styles, 150, function() { $(this).remove() });
    }
    return newElements
};
map_class.prototype.update = function() {
    this.orangeMarkElements = this.updateMarks(this.orangeMarks, this.orangeMarkElements, "http://i65.tinypic.com/xn8abn.png", 20, 20);
    this.greenMarkElements = this.updateMarks(this.greenMarks, this.greenMarkElements, "http://i68.tinypic.com/10mlx6g.png", 20, 20);
    this.greenCircleMarkElements = this.updateMarks(this.greenCircleMarks, this.greenCircleMarkElements, "http://i68.tinypic.com/rlb6si.png", 50, 50);
    this.blueMarkElements = this.updateMarks(this.blueMarks, this.blueMarkElements, "http://i63.tinypic.com/2qbhr2b.png", 20, 20);
    this.blackMarkElements = this.updateMarks([this.robotPoint], this.blackMarkElements, "http://i63.tinypic.com/2cf2hjk.png", 50, 50, this.robotAngle);
};
map_class.prototype.getPositionStyles = function(x, y, imgWidth, imgHeight) {
    relative_x = x - this.robotPoint.x;
    relative_y = y - this.robotPoint.y;
    let bottom = relative_x * this.meterSize + 250 - imgHeight / 2;
    let left = relative_y * this.meterSize + 250 - imgWidth / 2;

    return {bottom: bottom + "px", left: left + "px"}
};
