/*
 * JQ6500_COMMANDS.h
 *
 *  Created on: 5 Oct 2020
 *      Author: Z3r0
 */

#ifndef SRC_JQ6500_COMMANDS_H_
#define SRC_JQ6500_COMMANDS_H_

// Volume control
	uint8_t volumeValue = 15;	// Playback volume, 30 by default, value from 0 (minimum - mute) to 30 (maximum volume)
	uint8_t volumeUp   [] = { 0x7E, 0x02, 0x04, 0xEF };			// Volume increment
	uint8_t volumeDown [] = { 0x7E, 0x02, 0x05, 0xEF };			// Volume decrement
	uint8_t setVolume  [] = { 0x7E, 0x03, 0x06, 15, 0xEF };		// Set desired volume [0, 30], 15 by default
	uint8_t getVolume  [] = { 0x7E, 0x02, 0x43, 0xEF };			// Get current volume
// Playback control
	uint8_t alarmFile  [] = { 0x7E, 0x04, 0x03, 0x00, 0x01, 0xEF };		// Play the alarm at 7am / 11am / 1pm / 5pm
	uint8_t volumeTest [] = { 0x7E, 0x04, 0x03, 0x00, 0x05, 0xEF };		// Play the test file for volume control
	uint8_t Pause	   [] = { 0x7E, 0x02, 0x0E, 0xEF };					// Pause any playing music/sound file

#endif /* SRC_JQ6500_COMMANDS_H_ */
