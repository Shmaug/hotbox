$(".navbtn").on("mouseenter", function(){
    $(this).animate({"backgroundColor": "#444"},250);
});
$(".navbtn").on("mouseleave", function(){
    $(this).animate({"backgroundColor": "#000"},100);
});