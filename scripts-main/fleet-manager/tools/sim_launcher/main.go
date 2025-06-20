package main

import (
	"flag"
	"io/ioutil"
	"log"
	"os"
	"os/signal"
	"strconv"
	"time"

	"github.com/docker/docker/api/types/mount"
	"gopkg.in/yaml.v2"
)

// First port to connect simulator to
const jmavsimPort int = 4560

// First port to connect drone container to simulator
const droneSimPort int = 14540

// JMAVSim docker image
const jmavsimImage string = "fleetman/jmavsim"

// Drone module docker image
const droneImage string = "fleetman/drone"

func main() {
	var nDrones int
	var showInterface bool
	var cfgFile string
	var cfg []Config
	var launchDroneContainerDelay int
	hasToLaunchDroneContainer := false

	flag.IntVar(&nDrones, "n", 1, "Number of drones")
	flag.BoolVar(&showInterface, "i", false, "Show drone simulator GUI")
	flag.StringVar(&cfgFile, "c", "", "Config file")
	flag.IntVar(&launchDroneContainerDelay, "d", 5, "Delay in seconds between launching the jmavsim container and the drone container")
	flag.Parse()

	if len(cfgFile) > 0 {
		if nDrones > 1 {
			log.Println("Number of drones provided with -n flag will be ignored due to a configuration file being present")
		}
		cfgData, err := ioutil.ReadFile(cfgFile)
		handleErr(err)
		err = yaml.Unmarshal(cfgData, &cfg)
		handleErr(err)
		for i := 0; i < len(cfg); i++ {
			// Exit if no config is provided on this entry
			if cfg[i].isEmpty() {
				log.Fatal("Provided config file has empty drone configuration")
			}
			// Set default coordinates if none are provided
			if cfg[i].Simulator.Coords.isEmpty() {
				log.Println("Using default coords on config #", i+1)
				cfg[i].setDefaultSimCoords(i, cfg[i].Simulator.ShowGUI)
			}
			// Exit if drone doesn't have id
			if !cfg[i].Fleetman.isEmpty() {
				if cfg[i].Fleetman.ID == "" {
					log.Fatal("Config declares a drone without drone ID")
				}
				hasToLaunchDroneContainer = true
			}
		}
	} else {
		cfg = make([]Config, nDrones)
		for i := 0; i < nDrones; i++ {
			cfg[i].setDefaultSimCoords(i, showInterface)
		}
	}

	cliWrapper := InitializeContext()

	if !cliWrapper.imageExists(jmavsimImage) {
		log.Fatal("jMAVSim docker image is not built. Run ./build_docker_images.sh on the /build directory before proceeding.")
	}
	if !cliWrapper.imageExists(droneImage) {
		log.Fatal("Drone module docker image is not built. Run ./build_docker_images.sh on the /build directory before proceeding.")
	}

	var jMavSimContainerId string
	var fleetmanContainerId string

	c := make(chan os.Signal, 1)
	signal.Notify(c, os.Interrupt)
	go func() {
		// TODO understand this
		for sig := range c {
			if sig.String() == "interrupt" {
				cliWrapper.stopContainer(jMavSimContainerId)
				if len(fleetmanContainerId) > 0 {
					cliWrapper.stopContainer(fleetmanContainerId)
				}
				os.Exit(1)
			}
		}
	}()

	jMavSimContainerId = cliWrapper.createAndStartContainer(jmavsimImage, "jmavsim", nil,
		[]string{
			"DISPLAY=" + os.Getenv("DISPLAY"),
		}, []mount.Mount{
			{
				Type:   mount.TypeBind,
				Source: "/tmp/.X11-unix",
				Target: "/tmp/.X11-unix:ro",
			},
		})

	launchJmvasim(cliWrapper, jMavSimContainerId, cfg)

	if hasToLaunchDroneContainer {
		time.Sleep(time.Second * time.Duration(launchDroneContainerDelay))
		fleetmanContainerId = cliWrapper.createAndStartContainer(droneImage, "drones", []string{"bash"}, nil, nil)
		launchDronesContainer(cliWrapper, fleetmanContainerId, cfg)
	}

	cliWrapper.waitForContainer(jMavSimContainerId)
	cliWrapper.waitForContainer(fleetmanContainerId)
}

func launchJmvasim(cliWrapper CliWrapper, containerId string, cfg []Config) {
	cliWrapper.execContainer(containerId, []string{"./tools/sitl_multiple_run.sh", strconv.Itoa(len(cfg))}, nil, false)

	for i := 0; i < len(cfg); i++ {
		var port = jmavsimPort + i
		log.Println("Launched drone", i, "simulator on port", droneSimPort+i)
		cliWrapper.execContainer(containerId,
			[]string{"./tools/jmavsim_run.sh", "-p", strconv.Itoa(port), "-l", "&"},
			[]string{"HEADLESS=" + strconv.Itoa(btoi(!cfg[i].Simulator.ShowGUI)),
				"PX4_HOME_LAT=" + strconv.FormatFloat(cfg[i].Simulator.Coords.Lat, 'f', -1, 64),
				"PX4_HOME_LON=" + strconv.FormatFloat(cfg[i].Simulator.Coords.Lon, 'f', -1, 64),
				"PX4_HOME_ALT=" + strconv.FormatFloat(cfg[i].Simulator.Coords.Alt, 'f', -1, 64)},
			false)
	}
}

func launchDronesContainer(cliWrapper CliWrapper, containerId string, cfg []Config) {
	for i := 0; i < len(cfg); i++ {
		if !cfg[i].Fleetman.isEmpty() {
			cfgFile := "/ws/" + cfg[i].Fleetman.ID + "_cfg.yml"
			echoCfg := "echo '" + cfg[i].Fleetman.toString(droneSimPort+i) + "' >> " + cfgFile
			cliWrapper.execContainer(containerId, []string{"bash", "-c", echoCfg}, nil, true)
			go cliWrapper.execContainer(containerId, []string{"bash", "-c", "source install/setup.bash && chmod +x run_drone_controller.sh && ./run_drone_controller.sh " + cfg[i].Fleetman.ID + " " + cfgFile}, nil, true)
			log.Println("Launched controller for drone " + cfg[i].Fleetman.ID)
		}
	}
}
