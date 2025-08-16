#include "mp_sdk_audio.h"

using namespace gmpi;

class RwSMultiply final : public MpBase2
{
	AudioInPin pinInput1;
	AudioInPin pinInput2;
	AudioOutPin pinOutput;

public:
	RwSMultiply()
	{
		initializePin( pinInput1 );
		initializePin( pinInput2 );
		initializePin( pinOutput );
	}

	void subProcess( int sampleFrames )
	{
		// get pointers to in/output buffers.
		auto input1 = getBuffer(pinInput1);
		auto input2 = getBuffer(pinInput2);
		auto output = getBuffer(pinOutput);

		for( int s = sampleFrames; s > 0; --s )
		{
			// TODO: Signal processing goes here.

			*output = *input1 * *input2;

			// Increment buffer pointers.
			++input1;
			++input2;
			++output;
		}
	}

	void onSetPins() override
	{
		// Check which pins are updated.
		if( pinInput1.isStreaming() )
		{
		}
		if( pinInput2.isStreaming() )
		{
		}

		// Set state of output audio pins.
		pinOutput.setStreaming(true);

		// Set processing method.
		setSubProcess(&RwSMultiply::subProcess);
	}
};

namespace
{
	auto r = Register<RwSMultiply>::withId(L"My RwSMultiply");
}
