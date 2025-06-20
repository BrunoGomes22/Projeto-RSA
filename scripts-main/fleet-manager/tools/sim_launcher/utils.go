package main

import "log"

func btoi(b bool) int {
	if b {
		return 1
	}
	return 0
}

func handleErr(err error) {
	if err != nil {
		log.Fatal(err)
	}
}