// By hzFishy - 2026 - Do whatever you want with it.


#include "Data/SANMovementPathRequest.h"


FSANMovementPathRequest::FSANMovementPathRequest()
{
	
}

bool FSANMovementPathRequest::IsValid() const
{
	return CachedPathRequest.IsValid() && !CachedPathResult.IsEmpty();
}
