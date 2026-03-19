package frc.lib;

import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

public interface ClassFieldMapStringToInt {
	public static Map<String, Integer> getAsMap(Class<?> clazz) {
		return Arrays.stream(clazz.getDeclaredFields())
			.filter(f -> Modifier.isStatic(f.getModifiers()))
			.filter(f -> f.getType() == int.class)
			.peek(f -> f.setAccessible(true))
			.collect(
				LinkedHashMap::new,
				(map, f) -> {
					try {
						map.put(f.getName(), f.getInt(null));
					} catch (Exception e) {
						map.put(f.getName(), null);
					}
				},
				Map::putAll);
	}

	public static Map<String, Integer> getAsMap(Class<?> clazz, boolean sortByValue) {
		return sortByValue ? sortByValue(getAsMap(clazz)) : getAsMap(clazz);
	}

	// function to sort hashmap by values
	private static Map<String, Integer> sortByValue(Map<String, Integer> hm) {
		// Create a list from elements of Map
		List<Map.Entry<String, Integer>> list = new LinkedList<Map.Entry<String, Integer>>(
			hm.entrySet());

		// Sort the list
		Collections.sort(list, new Comparator<Map.Entry<String, Integer>>() {
			public int compare(Map.Entry<String, Integer> o1,
				Map.Entry<String, Integer> o2) {
				return (o1.getValue()).compareTo(o2.getValue());
			}
		});

		// put data from sorted list to hashmap
		HashMap<String, Integer> temp = new LinkedHashMap<String, Integer>();
		for (Map.Entry<String, Integer> aa : list) {
			temp.put(aa.getKey(), aa.getValue());
		}
		return temp;
	}

	public static void invalidateDuplicates(Class<?> clazz) {
		invalidateDuplicates(clazz, getAsMap(clazz));
	}

	public static void invalidateDuplicates(Class<?> clazz, Map<String, Integer> map) {
		Map<Integer, String> reverse = new HashMap<>();

		for (Map.Entry<String, Integer> entry : map.entrySet()) {
			String name = entry.getKey();
			Integer value = entry.getValue();

			if (value == null) {
				throw new IllegalArgumentException(
					"Null value detected in " + clazz.getName() +
						": '" + name);
			}

			if (reverse.containsKey(value)) {
				throw new RuntimeException(
					"Duplicate value detected in " + clazz.getName() +
						": '" + name + "' and '" + reverse.get(value) +
						"' both map to " + value);
			}

			reverse.put(value, name);
		}
	}
}
