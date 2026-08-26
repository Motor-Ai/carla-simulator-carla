// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla.h"
#include "TramPriorityZoneComponent.h"
#include "TrafficLightBase.h"
#include "TrafficLightComponent.h"
#include "TrafficLightGroup.h"
#include "TrafficLightController.h"

UTramPriorityZoneComponent::UTramPriorityZoneComponent()
{
  // Not calling SetBoxExtent() here: it triggers physics body-setup
  // creation (NewObject<UBodySetup>() with no name), which UE4 disallows
  // when called directly inside a component's own constructor during CDO
  // creation. Resize the box in the Details/Viewport panel after placing it.
  SetCollisionProfileName(TEXT("Trigger"));

  OnComponentBeginOverlap.AddDynamic(this, &UTramPriorityZoneComponent::OnTramBeginOverlap);
  OnComponentEndOverlap.AddDynamic(this, &UTramPriorityZoneComponent::OnTramEndOverlap);
}

void UTramPriorityZoneComponent::OnTramBeginOverlap(
    UPrimitiveComponent *OverlappedComp,
    AActor *OtherActor,
    UPrimitiveComponent *OtherComp,
    int32 OtherBodyIndex,
    bool bFromSweep,
    const FHitResult &SweepResult)
{
  if (!OtherActor || !OtherActor->ActorHasTag(TramActorTag))
  {
    return;
  }

  ++TramCount;
  if (TramCount == 1)
  {
    ActivatePriority();
  }
}

void UTramPriorityZoneComponent::OnTramEndOverlap(
    UPrimitiveComponent *OverlappedComp,
    AActor *OtherActor,
    UPrimitiveComponent *OtherComp,
    int32 OtherBodyIndex)
{
  if (!OtherActor || !OtherActor->ActorHasTag(TramActorTag))
  {
    return;
  }

  --TramCount;
  if (TramCount <= 0)
  {
    TramCount = 0;
    DeactivatePriority();
  }
}

bool UTramPriorityZoneComponent::ResolveReferences()
{
  if (bReferencesResolved)
  {
    return true;
  }

  if (!CarLightActor || !TramLightActor)
  {
    UE_LOG(LogCarla, Error,
        TEXT("UTramPriorityZoneComponent: CarLightActor/TramLightActor not set in the Details panel."));
    return false;
  }

  UTrafficLightComponent *CarComponent = CarLightActor->GetTrafficLightComponent();
  UTrafficLightComponent *TramComponent = TramLightActor->GetTrafficLightComponent();
  if (!CarComponent || !TramComponent)
  {
    return false;
  }

  ATrafficLightGroup *ResolvedCarGroup = CarComponent->GetGroup();
  ATrafficLightGroup *ResolvedTramGroup = TramComponent->GetGroup();
  UTrafficLightController *ResolvedTramController = TramComponent->GetController();
  if (!ResolvedCarGroup || !ResolvedTramGroup || !ResolvedTramController)
  {
    // CarLightActor/TramLightActor haven't finished registering with the
    // OpenDRIVE controller yet (registration happens on a retrying timer
    // shortly after BeginPlay) - try again next time this is called.
    return false;
  }

  CarGroup = ResolvedCarGroup;
  TramGroup = ResolvedTramGroup;
  TramController = ResolvedTramController;
  bReferencesResolved = true;
  return true;
}

void UTramPriorityZoneComponent::ActivatePriority()
{
  if (!ResolveReferences())
  {
    UE_LOG(LogCarla, Error,
        TEXT("UTramPriorityZoneComponent: CarGroup/TramGroup/TramController not resolved yet, cannot activate."));
    return;
  }
  TramGroup->RemoveController(TramController);
  CarGroup->AddController(TramController);
}

void UTramPriorityZoneComponent::DeactivatePriority()
{
  if (!ResolveReferences())
  {
    return;
  }
  CarGroup->RemoveController(TramController);
  TramGroup->AddController(TramController);
}
