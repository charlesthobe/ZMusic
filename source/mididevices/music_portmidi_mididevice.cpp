/*
** music_portmidi_mididevice.cpp
** Provides access to PortMIDI for crossplatform hardware MIDI playback
**
**---------------------------------------------------------------------------
** Copyright 2025 GZDoom Maintainers and Contributors
** All rights reserved.
**
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions
** are met:
**
** 1. Redistributions of source code must retain the above copyright
**    notice, this list of conditions and the following disclaimer.
** 2. Redistributions in binary form must reproduce the above copyright
**    notice, this list of conditions and the following disclaimer in the
**    documentation and/or other materials provided with the distribution.
** 3. The name of the author may not be used to endorse or promote products
**    derived from this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
** IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
** OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
** IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
** INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
** NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
** THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**---------------------------------------------------------------------------
**
*/

#ifdef USE_PORTMIDI

#include <atomic>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <thread>

#include <portmidi.h>
#include <porttime.h>

#include "mididevice.h"
#include "zmusic/mididefs.h"
#include "zmusic/mus2midi.h"

//==========================================================================
//
// PortMIDIDevice - PortMIDI implementation
//
// Based on CoreMIDIDevice
//
//==========================================================================

class PortMIDIDevice : public MIDIDevice
{
public:
	PortMIDIDevice(int deviceID, bool precache);
	~PortMIDIDevice();

	int Open() override;
	void Close() override;
	bool IsOpen() const override;
	int GetTechnology() const override;
	int SetTempo(int tempo) override;
	int SetTimeDiv(int timediv) override;
	int StreamOut(MidiHeader* data) override;
	int StreamOutSync(MidiHeader* data) override;
	int Resume() override;
	void Stop() override;
	bool Pause(bool paused) override;
	bool FakeVolume() override;
	void InitPlayback() override;
	void PrecacheInstruments(const uint16_t* instruments, int count) override;

protected:
	bool Precache;

	bool PullEvent();
	void PlayerLoop();

	// PortMidi internals
	PortMidiStream* Stream;
	int BufferSize;
	int Latency;
	PmDeviceID DeviceID;

	void PrepareTempo(uint32_t tempo);
	void PrepareLongMsg(uint8_t* long_msg);
	void PrepareShortMsg(uint32_t short_msg);
	void HandlePulledEvent();
	enum EventType { EVENT_TEMPO, EVENT_LONG_MESSAGE, EVENT_SHORT_MESSAGE, EVENT_NOP };
	struct PulledEvent
	{
		union
		{
			uint32_t tempo;
			uint8_t* longMsg;
			uint32_t shortMsg;
		} data;
		EventType type;
		uint32_t tick_delta;
	};
	PulledEvent PulledEvent;

	// Threading
	std::thread PlayerThread;
	std::atomic<bool> Exit;
	std::mutex Mutex;
	std::condition_variable ExitCond;

	// Timing
	int InitialTempo;
	int Tempo;
	int Division;

	// ZMusic MidiHeader data
	MidiHeader* Events; // Linked list of MIDI headers akin to win32 MIDIHDR
	uint32_t Position; // Current position in the MidiHeader buffer
	uint32_t PositionOffset;
};

//==========================================================================
//
// PortMIDIDevice :: Constructor
//
//==========================================================================

PortMIDIDevice::PortMIDIDevice(int deviceID, bool precache)
	: DeviceID(deviceID)
	, Stream(nullptr)
	, BufferSize(1024)
	, Latency(1)
	, InitialTempo(500000)      // Default: 120 BPM (500,000 µs per quarter note)
	, Division(100)       // Default PPQN
	, Events(nullptr)
	, Position(0)
	, Precache(precache)
{
	// Initialize PM
	PmError error = Pm_Initialize();
	if (error)
	{
		ZMusic_Printf(ZMUSIC_MSG_ERROR,"Couldn't initialize PortMIDI: %s", Pm_GetErrorText(error));
	}
}

//==========================================================================
//
// PortMIDIDevice :: Destructor
//
//==========================================================================

PortMIDIDevice::~PortMIDIDevice()
{
	Close();
	Pm_Terminate();
}

//==========================================================================
//
// PortMIDIDevice :: Open
//
// Opens the MIDI device and connects to the specified endpoint
//
//==========================================================================

int PortMIDIDevice::Open()
{
	if (Stream) { return 0; }

	PmError error;

	// Create PM output
	int outputDevice = DeviceID;
	if (DeviceID > Pm_CountDevices() - 1)
	{
		outputDevice = Pm_GetDefaultOutputDeviceID();
		ZMusic_Printf(ZMUSIC_MSG_ERROR,"Device index \"%d\" is invalid, using default \"%d\" \"%s\" instead.\n"
			, DeviceID, outputDevice, Pm_GetDeviceInfo(outputDevice)->name);
	}

	error = Pm_OpenOutput(&Stream, outputDevice, NULL, BufferSize, NULL, NULL, Latency);
	if (error)
	{
		ZMusic_Printf(ZMUSIC_MSG_ERROR,"Couldn't create PortMIDI output device: %s\n", Pm_GetErrorText(error));
	}

	return error;
}

//==========================================================================
//
// PortMIDIDevice :: Close
//
//==========================================================================

void PortMIDIDevice::Close()
{
	if (!Stream) { return; }

	// Stop player thread
	Stop();

	PmError error = Pm_Close(Stream);
	if (error)
	{
		ZMusic_Printf(ZMUSIC_MSG_ERROR,"Couldn't close PortMIDI stream: %s\n", Pm_GetErrorText(error));
	}
	Stream = nullptr;
}

//==========================================================================
//
// PortMIDIDevice :: IsOpen
//
//==========================================================================

bool PortMIDIDevice::IsOpen() const
{
	return Stream;
}

//==========================================================================
//
// PortMIDIDevice :: GetTechnology
//
//==========================================================================

int PortMIDIDevice::GetTechnology() const
{
	// Query if device is offline/virtual
	if (DeviceID == -1)
	{
		return MIDIDEV_SWSYNTH;
	}
	return MIDIDEV_MIDIPORT;
}

//==========================================================================
//
// PortMIDIDevice :: FakeVolume
//
// PortMIDI doesn't support volume control directly
//
//==========================================================================

bool PortMIDIDevice::FakeVolume()
{
	return true;  // No true volume control support, so fake volume
}

//==========================================================================
//
// PortMIDIDevice :: SetTempo
//
// Sets the playback tempo (microseconds per quarter note)
//
//==========================================================================

int PortMIDIDevice::SetTempo(int tempo)
{
	InitialTempo = tempo;
	return 0;
}

//==========================================================================
//
// PortMIDIDevice :: SetTimeDiv
//
// Sets the time division (PPQN - pulses per quarter note)
//
//==========================================================================

int PortMIDIDevice::SetTimeDiv(int timediv)
{
	Division = timediv > 0 ? timediv : 96;
	return 0;
}

//==========================================================================
//
// PortMIDIDevice :: PrecacheInstruments
//
// This is meant to mirror WinMIDIDevice::PrecacheInstruments
//
//==========================================================================

void PortMIDIDevice::PrecacheInstruments(const uint16_t* instruments, int count)
{
	// Setting snd_midiprecache to false disables this precaching, since it
	// does involve sleeping for more than a miniscule amount of time.
	if (!Precache)
	{
		return;
	}
	uint8_t bank[16] = {0};
	int i, chan;

	for (i = 0, chan = 0; i < count; ++i)
	{
		int instr = instruments[i] & 127;
		int banknum = (instruments[i] >> 7) & 127;
		int percussion = instruments[i] >> 14;

		if (percussion)
		{
			if (bank[9] != banknum)
			{
				Pm_WriteShort(Stream, 0, Pm_Message(MIDI_CTRLCHANGE | 9, 0, banknum)); //(MIDI_CTRLCHANGE | 9 | (0 << 8) | (banknum << 16));
				bank[9] = banknum;
			}
			Pm_WriteShort(Stream, 0, Pm_Message(MIDI_NOTEON | 9, instruments[i] & 0x7f, 1)); // (MIDI_NOTEON | 9 | ((instruments[i] & 0x7f) << 8) | (1 << 16));
		}
		else
		{ // Melodic
			if (bank[chan] != banknum)
			{
				Pm_WriteShort(Stream, 0, Pm_Message(MIDI_CTRLCHANGE | 9, 0, banknum)); //(MIDI_CTRLCHANGE | 9 | (0 << 8) | (banknum << 16));
				bank[chan] = banknum;
			}
			Pm_WriteShort(Stream, 0, Pm_Message(MIDI_PRGMCHANGE | chan, instruments[i], 0)); //(MIDI_PRGMCHANGE | chan | (instruments[i] << 8));
			Pm_WriteShort(Stream, 0, Pm_Message(MIDI_NOTEON | chan, 60, 1)); //(MIDI_NOTEON | chan | (60 << 8) | (1 << 16));
			if (++chan == 9)
			{ // Skip the percussion channel
				chan = 10;
			}
		}
		// Once we've got an instrument playing on each melodic channel, sleep to give
		// the driver time to load the instruments. Also do this for the final batch
		// of instruments.
		if (chan == 16 || i == count - 1)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(250));
			for (chan = 15; chan-- != 0; )
			{
				// Turn all notes off
				Pm_WriteShort(Stream, 0, Pm_Message(MIDI_CTRLCHANGE | chan, 123, 0)); //(MIDI_CTRLCHANGE | chan | (123 << 8));
			}
			// And now chan is back at 0, ready to start the cycle over.
		}
	}
	// Make sure all channels are set back to bank 0.
	for (i = 0; i < 16; ++i)
	{
		if (bank[i] != 0)
		{
			Pm_WriteShort(Stream, 0, Pm_Message(MIDI_CTRLCHANGE | 9, 0, 0)); //(MIDI_CTRLCHANGE | 9 | (0 << 8) | (0 << 16));
		}
	}
}

//==========================================================================
//
// PortMIDIDevice :: InitPlayback
//
// Initialize playback state
//
//==========================================================================

void PortMIDIDevice::InitPlayback()
{
	Exit.store(false, std::memory_order_relaxed);
}

//==========================================================================
//
// PortMIDIDevice :: Resume
//
// Start or resume playback
//
//==========================================================================

int PortMIDIDevice::Resume()
{
	if (!Stream || PlayerThread.joinable())
	{
		return -1;
	}
	Exit.store(false, std::memory_order_relaxed);
	PlayerThread = std::thread(&PortMIDIDevice::PlayerLoop, this);
	return 0;
}

//==========================================================================
//
// PortMIDIDevice :: Stop
//
// Stop playback
//
//==========================================================================

void PortMIDIDevice::Stop()
{
	Exit.store(true, std::memory_order_relaxed);
	ExitCond.notify_all();
	if (PlayerThread.joinable())
	{
		PlayerThread.join();
	}

	// Send All Notes Off and Reset All Controllers
	for (int channel = 0; channel < 16; ++channel)
	{
		Pm_WriteShort(Stream, 0, Pm_Message(MIDI_CTRLCHANGE | channel, 123, 0)); // Notes off
		Pm_WriteShort(Stream, 0, Pm_Message(MIDI_CTRLCHANGE | channel, 121, 0)); // Reset all controllers
	}
	// PortMidi is broken at least on linux, waiting until the event queue is empty can fix it sometimes.
	std::this_thread::sleep_for(std::chrono::milliseconds(120));
}

//==========================================================================
//
// PortMIDIDevice :: Pause
//
// We cannot pause so just always return false
//
//==========================================================================

bool PortMIDIDevice::Pause(bool paused)
{
	return false;
}

//==========================================================================
//
// PortMIDIDevice :: StreamOut
//
// Gets new midi buffers
//
//==========================================================================

int PortMIDIDevice::StreamOut(MidiHeader* header)
{
	header->lpNext = nullptr;
	if (Events == nullptr)
	{
		Events = header;
		Position = 0;
	}
	else
	{
		MidiHeader** p;
		for (p = &Events; *p != nullptr; p = &(*p)->lpNext)
		{ }
		*p = header;
	}
	return 0;
}

//==========================================================================
//
// PortMIDIDevice :: StreamOutSync
//
//==========================================================================

int PortMIDIDevice::StreamOutSync(MidiHeader* header)
{
	return StreamOut(header);
}

//==========================================================================
//
// PortMIDIDevice :: PullEvent
//
// Pulls next event from MidiHeader buffer
//
//==========================================================================

bool PortMIDIDevice::PullEvent()
{
	if (!Events && Callback)
	{	// No events in the current MidiHeader, request next buffer
		Callback(CallbackData);
	}

	if (!Events)
	{	// No events available to process.
		return false;
	}

	if (Position >= Events->dwBytesRecorded)
	{	// All events in the buffer were used, point to next buffer
		Events = Events->lpNext;
		Position = 0;
		if (Callback)
		{	// This ensures that we always have the maximum number of unused buffers (most likely 2) after 1 is used up.
			// omit this nested "if" block if you want to use up all buffers before requesting new buffers
			Callback(CallbackData);
		}
	}

	if (!Events)
	{	// No events in the new buffer
		return false;
	}

	uint32_t* event = (uint32_t*)(Events->lpData + Position);
	PulledEvent.tick_delta = event[0]; // First 4 bytes of event

	// Get event size to advance Position
	if (event[2] < 0x80000000) // Short message (event[2] is the combined status/data bytes)
	{
		PositionOffset = 12; // 4 bytes delta time, 4 bytes reserved, 4 bytes MIDI message (up to 3 bytes + padding)
	}
	else // Long message or meta-event (event[2] holds type and parameter length)
	{
		PositionOffset = 12 + ((MEVENT_EVENTPARM(event[2]) + 3) & ~3);
	}

	// Pulling event out of buffer
	switch (MEVENT_EVENTTYPE(event[2]))
	{
	case MEVENT_TEMPO:
		// Tempo change event, update our internal calculation for future events
		PrepareTempo(MEVENT_EVENTPARM(event[2]));
		break;
	case MEVENT_LONGMSG:
		{	// Long MIDI message (SysEx, etc.), data starts after event[3]
			int long_msg_len = MEVENT_EVENTPARM(event[2]);
			uint8_t* long_msg_data = (uint8_t*)&event[3];
			// Ensure valid sysex message
			if (long_msg_len > 2 && long_msg_data[0] == 0xF0 && long_msg_data[long_msg_len - 1] == 0xF7)
			{
				PrepareLongMsg(long_msg_data);
			}
			else
			{
				PulledEvent.type = EVENT_NOP;
			}
			break;
		}
	case MEVENT_SHORTMSG:
		{
			uint32_t msg;
			msg = Pm_Message(	(uint8_t)(event[2] & 0xff), // Status
								(uint8_t)((event[2] >> 8) & 0xff), // Data 1
								(uint8_t)((event[2] >> 16) & 0xff) ); // Data 2
			PrepareShortMsg(msg);
			break;
		}
	default:
		PulledEvent.type = EVENT_NOP;
	}

	// Indicate that an event was processed.
	return true;
}

//==========================================================================
//
// PortMIDIDevice :: PlayerLoop
//
// Main player thread loop - processes MIDI events from queue
//
//==========================================================================

void PortMIDIDevice::PlayerLoop()
{
	std::unique_lock<std::mutex> lock(Mutex);
	std::chrono::milliseconds buffer_step(40);

	Tempo = InitialTempo;
	// Initialize midi clock with current host time
	PmTimestamp buffer_timestamp = Pt_Time();

	// Process all available events and schedule them with PortMIDI
	while (!Exit.load(std::memory_order_relaxed))
	{
		if (!PullEvent())
		{
			ExitCond.wait_for(lock, buffer_step);
			continue;
		}

		// PortMidi works in milliseconds so devide by 1000, accurate to 0.5 milliseconds via promotion to double and rounding.
		PmTimestamp pulled_ev_timestamp = buffer_timestamp + round(PulledEvent.tick_delta * Tempo / Division / double(1000));

		auto time_until_pulled_ev = std::chrono::milliseconds(pulled_ev_timestamp - Pt_Time());
		auto schedule_time = time_until_pulled_ev - buffer_step;
		if (schedule_time >= buffer_step)
		{    // Try to keep buffered events under 2x buffer_step
			if (ExitCond.wait_for(lock, schedule_time) == std::cv_status::no_timeout)
			{
				continue;
			}
		}
		if (time_until_pulled_ev < std::chrono::milliseconds::zero())
		{	// Can be triggered on playback start.
			// Message shouldn't be shown by default like other midi backends here.
			ZMusic_Printf(ZMUSIC_MSG_DEBUG, "PortMidi backend underrun by %d milliseconds!\n", time_until_pulled_ev.count());
		}

		// Handle PulledEvent
		switch (PulledEvent.type)
		{
			case EVENT_TEMPO:
				Tempo = PulledEvent.data.tempo;
				break;
			case EVENT_LONG_MESSAGE:
				Pm_WriteSysEx(Stream, pulled_ev_timestamp, PulledEvent.data.longMsg);
				break;
			case EVENT_SHORT_MESSAGE:
				Pm_WriteShort(Stream, pulled_ev_timestamp, PulledEvent.data.shortMsg);
			case EVENT_NOP:
			default:
				;
		}
		buffer_timestamp = pulled_ev_timestamp;
		Position += PositionOffset;
	}
}

//==========================================================================
//
// PortMIDIDevice :: PrepareTempo, PrepareLongMsg and PrepareShortMsg
//
// Prepare pulled event to be handled later
//
//==========================================================================

void PortMIDIDevice::PrepareTempo(const uint32_t tempo)
{
	PulledEvent.type = EVENT_TEMPO;
	PulledEvent.data.tempo = tempo;
}
void PortMIDIDevice::PrepareLongMsg(uint8_t* long_msg)
{
	PulledEvent.type = EVENT_LONG_MESSAGE;
	PulledEvent.data.longMsg = long_msg;
}
void PortMIDIDevice::PrepareShortMsg(uint32_t short_msg)
{
	PulledEvent.type = EVENT_SHORT_MESSAGE;
	PulledEvent.data.shortMsg = short_msg;
}

//==========================================================================
//
// CreatePortMIDIDevice
//
// Factory function to create a PortMIDI device instance
//
//==========================================================================

MIDIDevice* CreatePortMIDIDevice(int mididevice)
{
	//return new PortMIDIDevice(mididevice);
	return new PortMIDIDevice(mididevice, miscConfig.snd_midiprecache);
}

#endif
