// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "CoreMinimal.h"
#include "Components/BoxComponent.h"
#include "TramPriorityZoneComponent.generated.h"

class ATrafficLightBase;
class ATrafficLightGroup;
class UTrafficLightController;

/// Attachable version of the tram priority trigger: add this component to
/// any one already-placed actor (e.g. one of your tram lamp Blueprints) via
/// "+Add Component", instead of placing a separate Actor in the level.
///
/// Detects a tram entering/leaving this component's box and temporarily
/// moves the tram's UTrafficLightController into the car junction's
/// ATrafficLightGroup (so it takes a turn in the same round-robin), then
/// moves it back once the tram has left. Multiple overlapping trams are
/// handled with a counter, so it only deactivates once every tram inside it
/// has left.
///
/// Only add ONE of these per junction, even if several lamp instances share
/// the same controller - multiple independent components all calling
/// AddController/RemoveController on the same shared Group/Controller will
/// desync each other's counters and duplicate entries in the Group's
/// Controllers array.
///
/// CarGroup/TramGroup/TramController are never set directly in the editor:
/// ATrafficLightGroup/UTrafficLightController are spawned at runtime by
/// ATrafficLightManager's registration, not placed in the level, so there is
/// nothing to reference at edit time. Instead this class references one
/// already-placed light actor on each side and resolves the actual Group/
/// Controller objects from it lazily, once registration has completed.
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class CARLA_API UTramPriorityZoneComponent : public UBoxComponent
{
  GENERATED_BODY()

public:
  UTramPriorityZoneComponent();

protected:
  UFUNCTION()
  void OnTramBeginOverlap(
      UPrimitiveComponent *OverlappedComp,
      AActor *OtherActor,
      UPrimitiveComponent *OtherComp,
      int32 OtherBodyIndex,
      bool bFromSweep,
      const FHitResult &SweepResult);

  UFUNCTION()
  void OnTramEndOverlap(
      UPrimitiveComponent *OverlappedComp,
      AActor *OtherActor,
      UPrimitiveComponent *OtherComp,
      int32 OtherBodyIndex);

private:
  void ActivatePriority();
  void DeactivatePriority();

  // Returns true once CarGroup/TramGroup/TramController have all been
  // resolved. Safe to call repeatedly - does nothing once already resolved,
  // and retries resolution (returning false) if the referenced actors
  // haven't finished registering with the OpenDRIVE controller yet.
  bool ResolveReferences();

  // Actor Tag a tram must have (set on the tram Blueprint's Class Defaults)
  // for this zone to recognize it. Using a tag rather than a C++ class cast
  // means the tram can be a pure Blueprint actor with no shared C++ base.
  UPROPERTY(Category = "Tram Priority", EditAnywhere)
  FName TramActorTag = "tram";

  // Any one already-placed light actor registered under the car junction's
  // controller (e.g. one of the phase18 lights). Used only to resolve
  // CarGroup at runtime - never touched itself.
  UPROPERTY(Category = "Tram Priority", EditAnywhere)
  ATrafficLightBase *CarLightActor;

  // Any one of this pole's already-placed tram lamp actors. Used to resolve
  // TramGroup and TramController at runtime - never touched itself.
  UPROPERTY(Category = "Tram Priority", EditAnywhere)
  ATrafficLightBase *TramLightActor;

  // Resolved lazily from CarLightActor/TramLightActor via ResolveReferences().
  UPROPERTY(Transient)
  ATrafficLightGroup *CarGroup;

  UPROPERTY(Transient)
  ATrafficLightGroup *TramGroup;

  UPROPERTY(Transient)
  UTrafficLightController *TramController;

  bool bReferencesResolved = false;

  UPROPERTY(Category = "Tram Priority", VisibleAnywhere)
  int32 TramCount = 0;
};
