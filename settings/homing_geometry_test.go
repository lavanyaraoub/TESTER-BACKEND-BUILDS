package settings

import (
	"encoding/json"
	"os"
	"path/filepath"
	"testing"
)

func TestHomingGeometryPersistenceAndInvalidation(t *testing.T) {
	path := filepath.Join(t.TempDir(), "settings.json")
	data := []byte(`{"A":{"homing_valid":true,"homing_apos":123,"jog_feed":"15","custom_field":42},"B":{"homing_apos":456}}`)
	if err := os.WriteFile(path, data, 0600); err != nil {
		t.Fatal(err)
	}
	settingsMutex.Lock()
	old := settingsRoot
	settingsRoot = SettingsRoot{"A": DriverSettings{HomingValid: true, HomingApos: 123}}
	settingsMutex.Unlock()
	t.Cleanup(func() { settingsMutex.Lock(); settingsRoot = old; settingsMutex.Unlock() })
	stamp := "mode4-test-geometry"
	if err := updateHomingMetadataAt(path, "A", &stamp); err != nil {
		t.Fatal(err)
	}
	if ds := GetDriverSettings("A"); ds.HomingGeometry != stamp || !ds.HomingValid || ds.HomingApos != 123 {
		t.Fatalf("cache changed incorrectly: %+v", ds)
	}
	if err := updateHomingMetadataAt(path, "A", nil); err != nil {
		t.Fatal(err)
	}
	if GetDriverSettings("A").HomingValid {
		t.Fatal("home was not invalidated")
	}
	data, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	var root map[string]map[string]interface{}
	if err := json.Unmarshal(data, &root); err != nil {
		t.Fatal(err)
	}
	if root["A"]["homing_valid"] != false || root["A"]["custom_field"] != float64(42) || root["B"]["homing_apos"] != float64(456) {
		t.Fatalf("unrelated settings lost: %s", data)
	}
	if err := updateHomingMetadataAt(path, "missing", &stamp); err == nil {
		t.Fatal("unknown axis accepted")
	}
}

func TestUISettingsCannotOverwriteHomingMetadata(t *testing.T) {
	path := filepath.Join(t.TempDir(), "settings.json")
	settingsMutex.Lock()
	old := settingsRoot
	settingsRoot = SettingsRoot{"A": DriverSettings{HomingValid: true, HomingApos: 123, HomingDriveXRatio: 20000, HomingGeometry: "verified"}}
	settingsMutex.Unlock()
	t.Cleanup(func() { settingsMutex.Lock(); settingsRoot = old; settingsMutex.Unlock() })
	body := []byte(`{"A":{"jog_feed":"25","homing_offset":"12","homing_valid":false,"homing_apos":999}}`)
	if err := writeSettingsPreservingHomingAt(path, body); err != nil {
		t.Fatal(err)
	}
	data, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	var root map[string]map[string]interface{}
	if err := json.Unmarshal(data, &root); err != nil {
		t.Fatal(err)
	}
	a := root["A"]
	if a["homing_valid"] != true || a["homing_apos"] != float64(123) || a["homing_geometry"] != "verified" || a["jog_feed"] != "25" || a["homing_offset"] != "12" {
		t.Fatalf("bad settings merge: %s", data)
	}
}
