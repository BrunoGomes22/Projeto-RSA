package main

import (
	"strconv"
)

type Config struct {
	Simulator SimulatorCfg `yaml:"simulator"`
	Fleetman  FleetmanCfg  `yaml:"fleetman"`
}

type SimulatorCfg struct {
	ShowGUI bool   `yaml:"showGUI"`
	Coords  Coords `yaml:"coords"`
}

type Coords struct {
	Lat float64 `yaml:"lat"`
	Lon float64 `yaml:"lon"`
	Alt float64 `yaml:"alt"`
}

type FleetmanCfg struct {
	ID               string  `yaml:"id"`
	TelemetryTopic   string  `yaml:"telemetryTopic"`
	CommandTopic     string  `yaml:"commandTopic"`
	StatusTopic      string  `yaml:"statusTopic"`
	TelemetryRateMs  int     `yaml:"telemetryRateMs"`
	AcceptanceRadius float64 `yaml:"acceptanceRadius"`
	TakeoffAltitude  float64 `yaml:"takeoffAltitude"`
	ReturnAltitude   float64 `yaml:"returnAltitude"`
	MaxSpeed         float64 `yaml:"maxSpeed"`
	MaxAltitude      float64 `yaml:"maxAltitude"`
}

func (cfg *Config) setDefaultSimCoords(n int, showInterface bool) {
	cfg.Simulator.Coords.Lat = 40.63349 + float64(n)*0.0001
	cfg.Simulator.Coords.Lon = -8.660815 + float64(n)*0.0001
	cfg.Simulator.Coords.Alt = 8
	cfg.Simulator.ShowGUI = showInterface
}

func (cfg Config) isEmpty() bool {
	return cfg.Fleetman.isEmpty() && cfg.Simulator.isEmpty()
}

func (cfg FleetmanCfg) isEmpty() bool {
	return cfg == FleetmanCfg{"", "", "", "", 0, 0, 0, 0, 0, 0}
}

func (cfg SimulatorCfg) isEmpty() bool {
	return cfg == SimulatorCfg{false, Coords{0, 0, 0}}
}

func (coords Coords) isEmpty() bool {
	return coords == Coords{0, 0, 0}
}

func (cfg FleetmanCfg) toString(port int) string {
	str := "port: udp://:" + strconv.Itoa(port)
	if cfg.TelemetryTopic != "" {
		str += "\ntelemetryTopic: " + cfg.TelemetryTopic
	}
	if cfg.CommandTopic != "" {
		str += "\ncommandTopic: " + cfg.CommandTopic
	}
	if cfg.StatusTopic != "" {
		str += "\nstatusTopic: " + cfg.StatusTopic
	}
	if cfg.TelemetryRateMs != 0 {
		str += "\ntelemetryRateMs: " + strconv.Itoa(cfg.TelemetryRateMs)
	}
	if cfg.AcceptanceRadius != 0 {
		str += "\nacceptanceRadius: " + strconv.FormatFloat(cfg.AcceptanceRadius, 'f', 6, 64)
	}
	if cfg.TakeoffAltitude != 0 {
		str += "\ntakeoffAltitude: " + strconv.FormatFloat(cfg.TakeoffAltitude, 'f', 6, 64)
	}
	if cfg.MaxSpeed != 0 {
		str += "\nmaxSpeed: " + strconv.FormatFloat(cfg.MaxSpeed, 'f', 6, 64)
	}
	if cfg.MaxAltitude != 0 {
		str += "\nmaxAltitude: " + strconv.FormatFloat(cfg.MaxAltitude, 'f', 6, 64)
	}
	return str
}
