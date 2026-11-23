package settings

import (
	"EtherCAT/helper"
	"encoding/json"
	"fmt"
	"os"
)

// Persist the geometry separately from capture: a crash between calibration
// and stamping leaves a mismatch, which startup treats as an invalid home.
func SaveHomingGeometry(name, geometry string) error {
	return updateHomingMetadata(name, &geometry)
}

func InvalidateHomingCalibration(name string) error {
	return updateHomingMetadata(name, nil)
}

func updateHomingMetadata(name string, geometry *string) error {
	return updateHomingMetadataAt(helper.AppendWDPath("/settings/settings.json"), name, geometry)
}

func updateHomingMetadataAt(path, name string, geometry *string) error {
	settingsMutex.Lock()
	defer settingsMutex.Unlock()
	data, err := os.ReadFile(path)
	if err != nil {
		return err
	}
	var root map[string]json.RawMessage
	if err := json.Unmarshal(data, &root); err != nil {
		return err
	}
	var fields map[string]json.RawMessage
	if raw, ok := root[name]; !ok {
		return fmt.Errorf("homing metadata: axis %q not found", name)
	} else if err := json.Unmarshal(raw, &fields); err != nil {
		return err
	}
	if fields == nil {
		return fmt.Errorf("homing metadata: axis %q must be an object", name)
	}
	if geometry == nil {
		fields["homing_valid"] = json.RawMessage("false")
	} else {
		fields["homing_geometry"], err = json.Marshal(*geometry)
		if err != nil {
			return err
		}
	}
	root[name], err = json.Marshal(fields)
	if err != nil {
		return err
	}
	data, err = json.MarshalIndent(root, "", "\t")
	if err != nil {
		return err
	}
	tmp := path + ".geometry.tmp"
	if err := os.WriteFile(tmp, data, 0644); err != nil {
		return err
	}
	if err := os.Rename(tmp, path); err != nil {
		return err
	}
	entry := settingsRoot[name]
	if geometry == nil {
		entry.HomingValid = false
	} else {
		entry.HomingGeometry = *geometry
	}
	settingsRoot[name] = entry
	return nil
}

// A UI settings save cannot replace calibration with a stale browser copy or
// drop fields an older frontend does not know. Home Offset remains editable;
// its derived home is recalculated by the existing settings-changed listener.
func WriteSettingsPreservingHoming(body []byte) error {
	return writeSettingsPreservingHomingAt(helper.AppendWDPath("/settings/settings.json"), body)
}

func writeSettingsPreservingHomingAt(path string, body []byte) error {
	settingsMutex.Lock()
	defer settingsMutex.Unlock()
	var root map[string]json.RawMessage
	if err := json.Unmarshal(body, &root); err != nil {
		return err
	}
	if root == nil {
		return fmt.Errorf("settings must be an object")
	}
	for name, saved := range settingsRoot {
		raw, present := root[name]
		if !present {
			continue
		}
		var fields map[string]json.RawMessage
		if err := json.Unmarshal(raw, &fields); err != nil {
			return err
		}
		if fields == nil {
			return fmt.Errorf("settings axis %q must be an object", name)
		}
		for key, value := range map[string]interface{}{
			"encoder_zero_apos": saved.EncoderZeroApos, "homing_apos": saved.HomingApos,
			"homing_drive_x_ratio": saved.HomingDriveXRatio, "homing_valid": saved.HomingValid,
			"homing_geometry": saved.HomingGeometry,
		} {
			encoded, err := json.Marshal(value)
			if err != nil {
				return err
			}
			fields[key] = encoded
		}
		encoded, err := json.Marshal(fields)
		if err != nil {
			return err
		}
		root[name] = encoded
	}
	out, err := json.MarshalIndent(root, "", "\t")
	if err != nil {
		return err
	}
	tmp := path + ".ui.tmp"
	if err := os.WriteFile(tmp, out, 0644); err != nil {
		return err
	}
	return os.Rename(tmp, path)
}
