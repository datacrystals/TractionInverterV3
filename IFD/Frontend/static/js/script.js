document.addEventListener('DOMContentLoaded', function() {
    // Initialize gauges
    var speedometer = new JustGage({
        id: "speedometer",
        value: 0,
        min: 0,
        max: 200,
        title: "Speedometer"
    });

    var linearGauge = new JustGage({
        id: "linear-gauge",
        value: 0,
        min: 0,
        max: 100,
        title: "Temperature"
    });

    // Fetch data from backend
    function fetchData() {
        fetch('/api/data')
            .then(response => response.json())
            .then(data => {
                speedometer.refresh(data.speed);
                linearGauge.refresh(data.temperature);
                document.getElementById('speed-readout').textContent = data.speed;
                document.getElementById('temp-readout').textContent = data.temperature;
            });
    }

    // Fetch data initially and every 5 seconds
    fetchData();
    setInterval(fetchData, 5000);

    // Screen switching
    document.getElementById('screen1Button').addEventListener('click', function() {
        document.getElementById('screen1').style.display = 'block';
        document.getElementById('screen2').style.display = 'none';
    });

    document.getElementById('screen2Button').addEventListener('click', function() {
        document.getElementById('screen1').style.display = 'none';
        document.getElementById('screen2').style.display = 'block';
    });
});
