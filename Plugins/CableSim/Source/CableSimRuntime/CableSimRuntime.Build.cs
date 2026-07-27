using UnrealBuildTool;

public class CableSimRuntime : ModuleRules
{
	public CableSimRuntime(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		CppStandard = CppStandardVersion.Cpp20;

		PublicDependencyModuleNames.AddRange(new[]
		{
			"Core",
			"CoreUObject",
			"Engine",
			"CableSimCore"
		});

		PrivateDependencyModuleNames.AddRange(new[]
		{
			"Chaos",
			"PhysicsCore"
		});
	}
}
