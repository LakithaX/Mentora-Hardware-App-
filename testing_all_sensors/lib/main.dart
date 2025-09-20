import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;
import 'dart:convert';
import 'dart:async';
import 'dart:math' as math;
import 'package:shared_preferences/shared_preferences.dart';
import 'package:google_generative_ai/google_generative_ai.dart';

// Data Models
class Achievement {
  final String id;
  final String title;
  final String description;
  final String icon;
  final int points;
  final bool isUnlocked;
  final DateTime? unlockedAt;

  Achievement({
    required this.id,
    required this.title,
    required this.description,
    required this.icon,
    required this.points,
    this.isUnlocked = false,
    this.unlockedAt,
  });

  Map<String, dynamic> toJson() => {
    'id': id,
    'title': title,
    'description': description,
    'icon': icon,
    'points': points,
    'isUnlocked': isUnlocked,
    'unlockedAt': unlockedAt?.millisecondsSinceEpoch,
  };

  factory Achievement.fromJson(Map<String, dynamic> json) => Achievement(
    id: json['id'],
    title: json['title'],
    description: json['description'],
    icon: json['icon'],
    points: json['points'],
    isUnlocked: json['isUnlocked'] ?? false,
    unlockedAt: json['unlockedAt'] != null
        ? DateTime.fromMillisecondsSinceEpoch(json['unlockedAt'])
        : null,
  );
}

class StudySession {
  final DateTime startTime;
  final DateTime? endTime;
  final String phase;
  final double productivityScore;
  final Map<String, dynamic> environmentalData;

  StudySession({
    required this.startTime,
    this.endTime,
    required this.phase,
    required this.productivityScore,
    required this.environmentalData,
  });
}

void main() {
  runApp(const MentoraApp());
}

class MentoraApp extends StatelessWidget {
  const MentoraApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Mentora Study Companion',
      theme: ThemeData(
        primarySwatch: Colors.blue,
        visualDensity: VisualDensity.adaptivePlatformDensity,
        useMaterial3: true,
        colorScheme: ColorScheme.fromSeed(
          seedColor: const Color(0xFF6C63FF),
          brightness: Brightness.light,
        ),
      ),
      darkTheme: ThemeData(
        useMaterial3: true,
        colorScheme: ColorScheme.fromSeed(
          seedColor: const Color(0xFF6C63FF),
          brightness: Brightness.dark,
        ),
      ),
      home: const MentoraController(),
    );
  }
}

class MentoraController extends StatefulWidget {
  const MentoraController({super.key});

  @override
  State<MentoraController> createState() => _MentoraControllerState();
}

class _MentoraControllerState extends State<MentoraController>
    with TickerProviderStateMixin {
  String esp32Ip = '192.168.1.100';

  // Gemini AI Configuration
  static const String _geminiApiKey = 'AIzaSyDUgVEzQlVM9lEhOxCVHq7CD69pzv6IuGg';
  late GenerativeModel _geminiModel;
  bool _geminiInitialized = false;

  // Robot status
  String currentEmotion = 'DEFAULT';
  String baseEmotion = 'DEFAULT';
  String connectionStatus = 'Disconnected';
  bool isConnected = false;
  bool isLoading = false;
  bool hasReaction = false;
  bool animationActive = false;
  String statusMessage = 'Ready';

  // Sensor data
  double lightLevel = 0.0;
  double heartRate = 0.0;
  double spO2 = 0.0;
  bool heartRateValid = false;
  bool touch1 = false;
  bool touch2 = false;
  bool tiltState = false;
  int soundLevel = 0;
  int touchCount1 = 0;
  int touchCount2 = 0;

  // Insights
  String lightStatus = 'unknown';
  String stressLevel = 'normal';
  String focusLevel = 'focused';
  String environmentStatus = 'quiet';
  String positionStatus = 'stable';
  int interactionCount = 0;
  String recommendation = 'Ready to study!';
  double studyScore = 75.0;

  // Additional status
  String signalStrength = '0';
  String uptime = '0';
  String lastCommand = '0';

  // AI Analytics & Learning System
  double productivityScore = 0.0;
  double concentrationIndex = 0.0;
  Map<String, dynamic> userPreferences = {};
  String studyPattern = 'unknown';
  List<String> personalizedRecommendations = [];
  Map<String, double> optimalLevels = {
    'light': 300.0,
    'temperature': 22.0,
    'sound': 40.0,
  };

  // Pomodoro Timer Integration
  bool isPomodoroActive = false;
  String pomodoroPhase = 'work'; // 'work', 'short_break', 'long_break'
  int workDuration = 25; // minutes
  int shortBreakDuration = 5; // minutes
  int longBreakDuration = 15; // minutes
  int completedPomodoros = 0;
  int currentPomodoroCount = 0;
  int remainingTime = 0; // seconds
  Timer? _pomodoroTimer;

  // Gamification & Motivation
  int totalPoints = 0;
  String currentLevel = 'Beginner';
  List<Achievement> achievements = [];
  int dailyGoal = 4; // pomodoros
  int dailyProgress = 0;
  int studyStreak = 0;
  DateTime lastStudyDate = DateTime.now();

  // Advanced Environmental Intelligence
  double temperature = 22.0;
  double airQuality = 100.0;
  bool motionDetected = false;
  bool voiceActivity = false;
  double comfortScore = 0.0;
  Map<String, double> environmentalProfile = {};

  // Enhanced Audio-Visual Feedback
  bool audioEnabled = true;
  bool visualFeedbackEnabled = true;
  String currentRGBColor = '#6C63FF';

  // AI Chat Feature
  List<Map<String, String>> aiChatHistory = [];
  TextEditingController chatController = TextEditingController();
  bool isAIChatVisible = false;

  Timer? _statusTimer;
  Timer? _sensorTimer;
  Timer? _analyticsTimer;
  Timer? _saveTimer;
  late AnimationController _pulseController;
  late AnimationController _breathingController;
  late AnimationController _waveController;
  late AnimationController _celebrationController;
  late Animation<double> _pulseAnimation;
  late Animation<double> _breathingAnimation;
  late Animation<double> _waveAnimation;
  late Animation<double> _celebrationAnimation;

  @override
  void initState() {
    super.initState();

    _initializeAnimations();
    _initializeTimers();
    _initializeAI();
    _loadUserData();
    _checkStatus();
  }

  void _initializeAnimations() {
    _pulseController = AnimationController(
      duration: const Duration(seconds: 2),
      vsync: this,
    );
    _pulseAnimation = Tween<double>(begin: 0.8, end: 1.2).animate(
      CurvedAnimation(parent: _pulseController, curve: Curves.easeInOut),
    );
    _pulseController.repeat(reverse: true);

    _breathingController = AnimationController(
      duration: const Duration(seconds: 4),
      vsync: this,
    );
    _breathingAnimation = Tween<double>(begin: 0.5, end: 1.0).animate(
      CurvedAnimation(parent: _breathingController, curve: Curves.easeInOut),
    );
    _breathingController.repeat(reverse: true);

    _waveController = AnimationController(
      duration: const Duration(seconds: 3),
      vsync: this,
    );
    _waveAnimation = Tween<double>(
      begin: 0.0,
      end: 2 * math.pi,
    ).animate(_waveController);
    _waveController.repeat();

    _celebrationController = AnimationController(
      duration: const Duration(seconds: 2),
      vsync: this,
    );
    _celebrationAnimation = Tween<double>(begin: 0.0, end: 1.0).animate(
      CurvedAnimation(parent: _celebrationController, curve: Curves.elasticOut),
    );
  }

  void _initializeTimers() {
    _statusTimer = Timer.periodic(const Duration(seconds: 3), (timer) {
      _checkStatus();
    });

    _sensorTimer = Timer.periodic(const Duration(seconds: 2), (timer) {
      _fetchSensorData();
      _fetchInsights();
    });

    _analyticsTimer = Timer.periodic(const Duration(seconds: 10), (timer) {
      _updateAnalytics();
      _fetchAnalytics();
      _sendEnvironmentalData();
    });

    _saveTimer = Timer.periodic(const Duration(seconds: 30), (timer) {
      _saveUserData();
    });
  }

  void _initializeAI() {
    _initializeAchievements();
    _calculateProductivityScore();
    _analyzeStudyPattern();
    _initializeGemini();
  }

  void _initializeGemini() {
    try {
      _geminiModel = GenerativeModel(
        model: 'gemini-1.5-flash',
        apiKey: _geminiApiKey,
        generationConfig: GenerationConfig(
          temperature: 0.7,
          topK: 40,
          topP: 0.95,
          maxOutputTokens: 1024,
        ),
      );
      _geminiInitialized = true;
      print('Gemini AI initialized successfully');
    } catch (e) {
      print('Failed to initialize Gemini AI: $e');
      _geminiInitialized = false;
    }
  }

  void _initializeAchievements() {
    achievements = [
      Achievement(
        id: 'first_pomodoro',
        title: 'First Steps',
        description: 'Complete your first Pomodoro session',
        icon: '🎯',
        points: 10,
      ),
      Achievement(
        id: 'focused_master',
        title: 'Focused Master',
        description: 'Maintain 90%+ focus for 5 consecutive sessions',
        icon: '🧠',
        points: 50,
      ),
      Achievement(
        id: 'streak_warrior',
        title: 'Streak Warrior',
        description: 'Study for 7 consecutive days',
        icon: '🔥',
        points: 100,
      ),
      Achievement(
        id: 'environment_expert',
        title: 'Environment Expert',
        description: 'Optimize your study environment 10 times',
        icon: '🌡️',
        points: 75,
      ),
      Achievement(
        id: 'night_owl',
        title: 'Night Owl',
        description: 'Complete 10 study sessions after 10 PM',
        icon: '🦉',
        points: 60,
      ),
      Achievement(
        id: 'early_bird',
        title: 'Early Bird',
        description: 'Complete 10 study sessions before 8 AM',
        icon: '🐦',
        points: 60,
      ),
    ];
  }

  @override
  void dispose() {
    _statusTimer?.cancel();
    _sensorTimer?.cancel();
    _analyticsTimer?.cancel();
    _pomodoroTimer?.cancel();
    _saveTimer?.cancel();
    _pulseController.dispose();
    _breathingController.dispose();
    _waveController.dispose();
    _celebrationController.dispose();
    _saveUserData(); // Save data before disposing
    super.dispose();
  }

  Future<void> _checkStatus() async {
    try {
      final response = await http
          .get(
            Uri.parse('http://$esp32Ip/status'),
            headers: {'Content-Type': 'application/json'},
          )
          .timeout(const Duration(seconds: 5));

      if (response.statusCode == 200) {
        final data = json.decode(response.body);
        setState(() {
          currentEmotion = data['emotion'] ?? 'DEFAULT';
          baseEmotion = data['base_emotion'] ?? 'DEFAULT';
          hasReaction = data['has_reaction'] ?? false;
          animationActive = data['animation_active'] ?? false;
          isConnected = data['wifi_connected'] ?? false;
          signalStrength = data['signal_strength']?.toString() ?? '0';
          uptime = _formatUptime(data['uptime'] ?? 0);
          lastCommand = _formatTimestamp(data['last_command'] ?? 0);
          connectionStatus = isConnected
              ? 'Connected'
              : 'ESP32 WiFi Disconnected';
        });
      }
    } catch (e) {
      setState(() {
        isConnected = false;
        connectionStatus = 'Connection Failed';
      });
    }
  }

  Future<void> _fetchSensorData() async {
    if (!isConnected) return;

    try {
      final response = await http
          .get(Uri.parse('http://$esp32Ip/sensors'))
          .timeout(const Duration(seconds: 5));

      if (response.statusCode == 200) {
        final data = json.decode(response.body);
        setState(() {
          // Basic sensors
          lightLevel = (data['light_level'] ?? 0).toDouble();
          heartRate = (data['heart_rate'] ?? 0).toDouble();
          spO2 = (data['spo2'] ?? 0).toDouble();
          heartRateValid = data['heart_rate_valid'] ?? false;
          touch1 = data['touch_1'] ?? false;
          touch2 = data['touch_2'] ?? false;
          tiltState = data['tilt_state'] ?? false;
          soundLevel = data['sound_level'] ?? 0;
          touchCount1 = data['touch_count_1'] ?? 0;
          touchCount2 = data['touch_count_2'] ?? 0;

          // Advanced Environmental Intelligence
          temperature = (data['temperature'] ?? 22.0).toDouble();
          airQuality = (data['air_quality'] ?? 100.0).toDouble();
          motionDetected = data['motion_detected'] ?? false;
          voiceActivity = data['voice_activity'] ?? false;

          // Calculate comfort score
          comfortScore = _calculateComfortScore();

          // Update environmental profile
          _updateEnvironmentalProfile();
        });
      }
    } catch (e) {
      print('Sensor data fetch error: $e');
    }
  }

  void _updateEnvironmentalProfile() {
    // Build environmental profile based on current conditions
    environmentalProfile['light_optimal'] =
        lightLevel >= 200 && lightLevel <= 500;
    environmentalProfile['temperature_optimal'] =
        temperature >= 20 && temperature <= 24;
    environmentalProfile['sound_optimal'] =
        soundLevel >= 30 && soundLevel <= 60;
    environmentalProfile['air_quality_good'] = airQuality >= 80;
    environmentalProfile['motion_stable'] = !motionDetected;
    environmentalProfile['voice_quiet'] = !voiceActivity;

    // Calculate overall environmental score
    int optimalCount = environmentalProfile.values
        .where((v) => v == true)
        .length;
    environmentalProfile['overall_score'] =
        (optimalCount / environmentalProfile.length) * 100;
  }

  Future<void> _fetchInsights() async {
    if (!isConnected) return;

    try {
      final response = await http
          .get(Uri.parse('http://$esp32Ip/insights'))
          .timeout(const Duration(seconds: 5));

      if (response.statusCode == 200) {
        final data = json.decode(response.body);
        setState(() {
          lightStatus = data['light_status'] ?? 'unknown';
          stressLevel = data['stress_level'] ?? 'normal';
          focusLevel = data['focus_level'] ?? 'focused';
          environmentStatus = data['environment_status'] ?? 'quiet';
          positionStatus = data['position_status'] ?? 'stable';
          interactionCount = data['interaction_count'] ?? 0;
          recommendation = data['recommendation'] ?? 'Ready to study!';
          studyScore = (data['study_score'] ?? 75.0).toDouble();
        });
      }
    } catch (e) {
      print('Insights fetch error: $e');
    }
  }

  // New API Endpoints for Advanced Features
  Future<void> _fetchAnalytics() async {
    if (!isConnected) return;

    try {
      final response = await http
          .get(Uri.parse('http://$esp32Ip/analytics'))
          .timeout(const Duration(seconds: 5));

      if (response.statusCode == 200) {
        final data = json.decode(response.body);
        setState(() {
          productivityScore = (data['productivity_score'] ?? productivityScore)
              .toDouble();
          concentrationIndex =
              (data['concentration_index'] ?? concentrationIndex).toDouble();
          studyPattern = data['study_pattern'] ?? studyPattern;

          // Update personalized recommendations from server
          if (data['recommendations'] != null) {
            personalizedRecommendations = List<String>.from(
              data['recommendations'],
            );
          }
        });
      }
    } catch (e) {
      print('Analytics fetch error: $e');
    }
  }

  Future<void> _sendPomodoroCommand(String command) async {
    if (!isConnected) return;

    try {
      final response = await http
          .post(
            Uri.parse('http://$esp32Ip/pomodoro'),
            headers: {'Content-Type': 'application/json'},
            body: json.encode({
              'command': command,
              'phase': pomodoroPhase,
              'remaining_time': remainingTime,
            }),
          )
          .timeout(const Duration(seconds: 5));

      if (response.statusCode == 200) {
        final data = json.decode(response.body);
        // Handle server response if needed
        print('Pomodoro command sent: $command');
      }
    } catch (e) {
      print('Pomodoro command error: $e');
    }
  }

  Future<void> _sendEnvironmentalData() async {
    if (!isConnected) return;

    try {
      final response = await http
          .post(
            Uri.parse('http://$esp32Ip/environment'),
            headers: {'Content-Type': 'application/json'},
            body: json.encode({
              'temperature': temperature,
              'air_quality': airQuality,
              'motion_detected': motionDetected,
              'voice_activity': voiceActivity,
              'comfort_score': comfortScore,
              'environmental_profile': environmentalProfile,
            }),
          )
          .timeout(const Duration(seconds: 5));

      if (response.statusCode == 200) {
        print('Environmental data sent successfully');
      }
    } catch (e) {
      print('Environmental data send error: $e');
    }
  }

  String _formatUptime(int milliseconds) {
    final duration = Duration(milliseconds: milliseconds);
    final hours = duration.inHours;
    final minutes = duration.inMinutes.remainder(60);
    final seconds = duration.inSeconds.remainder(60);
    return '${hours}h ${minutes}m ${seconds}s';
  }

  String _formatTimestamp(int timestamp) {
    if (timestamp == 0) return 'Never';
    final now = DateTime.now().millisecondsSinceEpoch;
    final diff = Duration(milliseconds: now - timestamp);
    if (diff.inSeconds < 60) return '${diff.inSeconds}s ago';
    if (diff.inMinutes < 60) return '${diff.inMinutes}m ago';
    return '${diff.inHours}h ago';
  }

  // AI Analytics & Learning System Methods
  void _calculateProductivityScore() {
    // Calculate productivity based on focus, comfort, and interactions
    double focusScore = _getFocusScore();
    double comfortScore = _calculateComfortScore();
    double interactionScore = _getInteractionScore();

    productivityScore =
        (focusScore * 0.4 + comfortScore * 0.3 + interactionScore * 0.3);
    productivityScore = math.max(0, math.min(100, productivityScore));

    _updateLevel();
  }

  double _getFocusScore() {
    // Analyze focus based on environmental factors
    double lightScore = _getLightScore();
    double soundScore = _getSoundScore();
    double positionScore = tiltState ? 50.0 : 100.0;

    return (lightScore + soundScore + positionScore) / 3;
  }

  double _getLightScore() {
    if (lightLevel < 50) return 30.0; // Too dark
    if (lightLevel > 1000) return 40.0; // Too bright
    if (lightLevel >= 200 && lightLevel <= 500) return 100.0; // Optimal
    return 70.0; // Acceptable
  }

  double _getSoundScore() {
    if (soundLevel < 30) return 100.0; // Quiet
    if (soundLevel > 80) return 20.0; // Too noisy
    return 80.0; // Moderate
  }

  double _calculateComfortScore() {
    double temperatureScore = _getTemperatureScore();
    double airQualityScore = airQuality;
    double ergonomicsScore = tiltState ? 60.0 : 100.0;

    return (temperatureScore + airQualityScore + ergonomicsScore) / 3;
  }

  double _getTemperatureScore() {
    if (temperature < 18 || temperature > 26) return 30.0;
    if (temperature >= 20 && temperature <= 24) return 100.0;
    return 70.0;
  }

  double _getInteractionScore() {
    // Reward positive interactions, penalize distractions
    double touchScore = (touchCount1 + touchCount2) > 0 ? 80.0 : 100.0;
    double heartRateScore = heartRateValid ? 90.0 : 70.0;

    return (touchScore + heartRateScore) / 2;
  }

  void _analyzeStudyPattern() {
    final now = DateTime.now();
    final hour = now.hour;

    if (hour >= 5 && hour < 12) {
      studyPattern = 'morning_person';
    } else if (hour >= 12 && hour < 18) {
      studyPattern = 'afternoon_focused';
    } else if (hour >= 18 && hour < 22) {
      studyPattern = 'evening_productive';
    } else {
      studyPattern = 'night_owl';
    }

    // Enhanced AI pattern analysis
    _analyzeAIStudyPattern();
  }

  Future<void> _analyzeAIStudyPattern() async {
    if (!_geminiInitialized) return;

    try {
      final prompt =
          '''
Analyze this student's study pattern and provide insights:

STUDY HISTORY:
- Total Pomodoros Completed: $completedPomodoros
- Study Streak: $studyStreak days
- Current Level: $currentLevel
- Productivity Score: ${productivityScore.toInt()}%
- Concentration Index: ${concentrationIndex.toInt()}%

RECENT ACTIVITY:
- Daily Progress: $dailyProgress/$dailyGoal pomodoros
- Current Time: ${DateTime.now().hour}:${DateTime.now().minute.toString().padLeft(2, '0')}
- Study Pattern: $studyPattern

ENVIRONMENTAL PREFERENCES:
- Optimal Light: ${optimalLevels['light']!.toInt()} lux
- Optimal Temperature: ${optimalLevels['temperature']!.toInt()}°C
- Optimal Sound: ${optimalLevels['sound']!.toInt()} dB

Based on this data, provide:
1. A refined study pattern classification (morning_person, afternoon_focused, evening_productive, night_owl, or mixed)
2. Optimal study times for this student
3. Environmental preferences summary

Respond in JSON format:
{
  "pattern": "refined_pattern",
  "optimal_times": ["time1", "time2"],
  "preferences": "brief_summary"
}
''';

      final response = await _geminiModel.generateContent([
        Content.text(prompt),
      ]);

      if (response.text != null) {
        _parseAIStudyPattern(response.text!);
      }
    } catch (e) {
      print('AI study pattern analysis failed: $e');
    }
  }

  void _parseAIStudyPattern(String aiResponse) {
    try {
      // Extract JSON from response
      final jsonMatch = RegExp(r'\{[^}]*\}').firstMatch(aiResponse);
      if (jsonMatch != null) {
        final jsonStr = jsonMatch.group(0)!;
        final data = jsonDecode(jsonStr);

        setState(() {
          studyPattern = data['pattern'] ?? studyPattern;
          // Store additional insights for future use
          userPreferences['ai_optimal_times'] = data['optimal_times'];
          userPreferences['ai_preferences'] = data['preferences'];
        });
      }
    } catch (e) {
      print('Failed to parse AI study pattern: $e');
    }
  }

  void _updateAnalytics() {
    _calculateProductivityScore();
    _updateConcentrationIndex();
    _learnUserPreferences();
    _generateRecommendations();
    _checkAchievements();
  }

  void _updateConcentrationIndex() {
    // Calculate concentration consistency over time
    double currentFocus = _getFocusScore();
    concentrationIndex = (concentrationIndex * 0.9) + (currentFocus * 0.1);
  }

  void _learnUserPreferences() {
    // Learn optimal levels based on productivity
    if (productivityScore > 80) {
      optimalLevels['light'] =
          (optimalLevels['light']! * 0.9) + (lightLevel * 0.1);
      optimalLevels['temperature'] =
          (optimalLevels['temperature']! * 0.9) + (temperature * 0.1);
      optimalLevels['sound'] =
          (optimalLevels['sound']! * 0.9) + (soundLevel * 0.1);
    }
  }

  void _generateRecommendations() {
    personalizedRecommendations.clear();

    // Basic rule-based recommendations
    if (lightLevel < optimalLevels['light']! - 50) {
      personalizedRecommendations.add('💡 Increase lighting for better focus');
    }
    if (temperature < optimalLevels['temperature']! - 2) {
      personalizedRecommendations.add('🌡️ Consider warming up the room');
    }
    if (soundLevel > optimalLevels['sound']! + 20) {
      personalizedRecommendations.add(
        '🔇 Reduce noise for better concentration',
      );
    }
    if (productivityScore < 60) {
      personalizedRecommendations.add('🧘 Take a short break to refresh');
    }
    if (studyStreak > 0) {
      personalizedRecommendations.add('🔥 Great streak! Keep it up!');
    }

    // Enhanced AI recommendations with Gemini
    _generateAIRecommendations();
  }

  Future<void> _generateAIRecommendations() async {
    if (!_geminiInitialized) return;

    try {
      final prompt = _buildAIRecommendationPrompt();
      final response = await _geminiModel.generateContent([
        Content.text(prompt),
      ]);

      if (response.text != null) {
        final aiRecommendations = _parseAIRecommendations(response.text!);
        personalizedRecommendations.addAll(aiRecommendations);

        // Limit to 5 recommendations total
        if (personalizedRecommendations.length > 5) {
          personalizedRecommendations = personalizedRecommendations
              .take(5)
              .toList();
        }
      }
    } catch (e) {
      print('AI recommendation generation failed: $e');
    }
  }

  String _buildAIRecommendationPrompt() {
    return '''
You are an AI Study Companion analyzing a student's study environment and productivity data. Provide 2-3 personalized, actionable recommendations based on this data:

STUDY SESSION DATA:
- Productivity Score: ${productivityScore.toInt()}%
- Concentration Index: ${concentrationIndex.toInt()}%
- Study Pattern: $studyPattern
- Current Level: $currentLevel
- Study Streak: $studyStreak days
- Completed Pomodoros Today: $dailyProgress/$dailyGoal

ENVIRONMENTAL CONDITIONS:
- Light Level: ${lightLevel.toInt()} lux (optimal: 200-500)
- Temperature: ${temperature.toInt()}°C (optimal: 20-24°C)
- Sound Level: $soundLevel dB (optimal: 30-60)
- Air Quality: ${airQuality.toInt()}% (good: >80%)
- Motion Detected: $motionDetected
- Voice Activity: $voiceActivity
- Comfort Score: ${comfortScore.toInt()}%

SENSOR DATA:
- Heart Rate: ${heartRateValid ? '${heartRate.toInt()} bpm' : 'Invalid'}
- SpO2: ${spO2.toInt()}%
- Touch Interactions: ${touchCount1 + touchCount2}
- Tilt State: $tiltState

CURRENT TIME: ${DateTime.now().hour}:${DateTime.now().minute.toString().padLeft(2, '0')}

Provide 2-3 specific, actionable recommendations to improve study productivity. Be encouraging and motivational. Format each recommendation with an appropriate emoji and keep them concise (under 50 characters each).

Examples:
- "🎯 Try 5-minute meditation before studying"
- "🌡️ Adjust room temperature to 22°C"
- "🔇 Use noise-canceling headphones"

Recommendations:
''';
  }

  List<String> _parseAIRecommendations(String aiResponse) {
    final lines = aiResponse.split('\n');
    final recommendations = <String>[];

    for (final line in lines) {
      final trimmed = line.trim();
      if (trimmed.isNotEmpty &&
          (trimmed.startsWith('-') ||
              trimmed.startsWith('•') ||
              trimmed.startsWith('*')) &&
          trimmed.length < 60) {
        // Remove bullet points and clean up
        final clean = trimmed.replaceAll(RegExp(r'^[-•*]\s*'), '').trim();
        if (clean.isNotEmpty) {
          recommendations.add(clean);
        }
      }
    }

    return recommendations.take(3).toList();
  }

  void _updateLevel() {
    if (totalPoints >= 500) {
      currentLevel = 'Master';
    } else if (totalPoints >= 200) {
      currentLevel = 'Dedicated';
    } else if (totalPoints >= 50) {
      currentLevel = 'Focused';
    } else {
      currentLevel = 'Beginner';
    }
  }

  void _checkAchievements() {
    for (var achievement in achievements) {
      if (!achievement.isUnlocked) {
        bool shouldUnlock = false;

        switch (achievement.id) {
          case 'first_pomodoro':
            shouldUnlock = completedPomodoros >= 1;
            break;
          case 'focused_master':
            shouldUnlock = concentrationIndex >= 90 && completedPomodoros >= 5;
            break;
          case 'streak_warrior':
            shouldUnlock = studyStreak >= 7;
            break;
          case 'environment_expert':
            shouldUnlock = _getEnvironmentOptimizations() >= 10;
            break;
          case 'night_owl':
            shouldUnlock = _getNightSessions() >= 10;
            break;
          case 'early_bird':
            shouldUnlock = _getMorningSessions() >= 10;
            break;
        }

        if (shouldUnlock) {
          _unlockAchievement(achievement);
        }
      }
    }
  }

  void _unlockAchievement(Achievement achievement) {
    setState(() {
      achievement = Achievement(
        id: achievement.id,
        title: achievement.title,
        description: achievement.description,
        icon: achievement.icon,
        points: achievement.points,
        isUnlocked: true,
        unlockedAt: DateTime.now(),
      );

      totalPoints += achievement.points;
      _celebrationController.forward().then((_) {
        _celebrationController.reverse();
      });
    });

    _showAchievementDialog(achievement);
  }

  int _getEnvironmentOptimizations() {
    // Count how many times user optimized environment
    return userPreferences['optimizations'] ?? 0;
  }

  int _getNightSessions() {
    return userPreferences['night_sessions'] ?? 0;
  }

  int _getMorningSessions() {
    return userPreferences['morning_sessions'] ?? 0;
  }

  // Pomodoro Timer Methods
  void _startPomodoro() {
    if (isPomodoroActive) return;

    setState(() {
      isPomodoroActive = true;
      pomodoroPhase = 'work';
      remainingTime = workDuration * 60;
    });

    _pomodoroTimer = Timer.periodic(const Duration(seconds: 1), (timer) {
      if (remainingTime > 0) {
        setState(() {
          remainingTime--;
        });
      } else {
        _completePomodoroPhase();
      }
    });

    _sendEmotion('FOCUSED');
    _sendPomodoroCommand('start');
    _playNotificationSound('work_start');
  }

  void _pausePomodoro() {
    _pomodoroTimer?.cancel();
    setState(() {
      isPomodoroActive = false;
    });
    _sendPomodoroCommand('pause');
  }

  void _stopPomodoro() {
    _pomodoroTimer?.cancel();
    setState(() {
      isPomodoroActive = false;
      pomodoroPhase = 'work';
      remainingTime = 0;
    });
    _sendPomodoroCommand('stop');
  }

  void _completePomodoroPhase() {
    _pomodoroTimer?.cancel();

    if (pomodoroPhase == 'work') {
      setState(() {
        completedPomodoros++;
        currentPomodoroCount++;
        dailyProgress++;
        totalPoints += 10;
        pomodoroPhase = currentPomodoroCount % 4 == 0
            ? 'long_break'
            : 'short_break';
        remainingTime = pomodoroPhase == 'long_break'
            ? longBreakDuration * 60
            : shortBreakDuration * 60;
      });

      _sendEmotion('HAPPY');
      _playNotificationSound('work_complete');
      _checkAchievements();
    } else {
      setState(() {
        pomodoroPhase = 'work';
        remainingTime = workDuration * 60;
      });

      _sendEmotion('READY');
      _playNotificationSound('break_end');
    }

    if (isPomodoroActive) {
      _pomodoroTimer = Timer.periodic(const Duration(seconds: 1), (timer) {
        if (remainingTime > 0) {
          setState(() {
            remainingTime--;
          });
        } else {
          _completePomodoroPhase();
        }
      });
    }
  }

  String _formatTime(int seconds) {
    final minutes = seconds ~/ 60;
    final remainingSeconds = seconds % 60;
    return '${minutes.toString().padLeft(2, '0')}:${remainingSeconds.toString().padLeft(2, '0')}';
  }

  // Data Persistence Methods
  Future<void> _loadUserData() async {
    final prefs = await SharedPreferences.getInstance();

    setState(() {
      totalPoints = prefs.getInt('total_points') ?? 0;
      completedPomodoros = prefs.getInt('completed_pomodoros') ?? 0;
      studyStreak = prefs.getInt('study_streak') ?? 0;
      dailyProgress = prefs.getInt('daily_progress') ?? 0;
      currentLevel = prefs.getString('current_level') ?? 'Beginner';

      // Load achievements
      final achievementsJson = prefs.getStringList('achievements');
      if (achievementsJson != null) {
        achievements = achievementsJson
            .map((json) => Achievement.fromJson(jsonDecode(json)))
            .toList();
      }

      // Load user preferences
      final preferencesJson = prefs.getString('user_preferences');
      if (preferencesJson != null) {
        userPreferences = jsonDecode(preferencesJson);
      }
    });
  }

  Future<void> _saveUserData() async {
    final prefs = await SharedPreferences.getInstance();

    await prefs.setInt('total_points', totalPoints);
    await prefs.setInt('completed_pomodoros', completedPomodoros);
    await prefs.setInt('study_streak', studyStreak);
    await prefs.setInt('daily_progress', dailyProgress);
    await prefs.setString('current_level', currentLevel);

    // Save achievements
    final achievementsJson = achievements
        .map((achievement) => jsonEncode(achievement.toJson()))
        .toList();
    await prefs.setStringList('achievements', achievementsJson);

    // Save user preferences
    await prefs.setString('user_preferences', jsonEncode(userPreferences));
  }

  // Audio-Visual Feedback Methods
  void _playNotificationSound(String type) {
    if (!audioEnabled) return;

    // Generate AI-powered motivational messages
    _generateAIMotivationalMessage(type).then((message) {
      if (message.isNotEmpty) {
        _showNotification(message, _getNotificationColor(type));
      } else {
        // Fallback to basic messages
        _showBasicNotification(type);
      }
    });
  }

  void _showBasicNotification(String type) {
    switch (type) {
      case 'work_start':
        _showNotification('🎯 Work session started!', Colors.green);
        break;
      case 'work_complete':
        _showNotification('🎉 Pomodoro completed!', Colors.blue);
        break;
      case 'break_end':
        _showNotification('⏰ Break time is over!', Colors.orange);
        break;
    }
  }

  Color _getNotificationColor(String type) {
    switch (type) {
      case 'work_start':
        return Colors.green;
      case 'work_complete':
        return Colors.blue;
      case 'break_end':
        return Colors.orange;
      default:
        return Colors.blue;
    }
  }

  Future<String> _generateAIMotivationalMessage(String type) async {
    if (!_geminiInitialized) return '';

    try {
      final prompt = _buildMotivationalPrompt(type);
      final response = await _geminiModel.generateContent([
        Content.text(prompt),
      ]);

      if (response.text != null) {
        return _parseMotivationalMessage(response.text!);
      }
    } catch (e) {
      print('AI motivational message generation failed: $e');
    }

    return '';
  }

  String _buildMotivationalPrompt(String type) {
    final basePrompt =
        '''
You are an AI Study Companion providing motivational messages. Generate a short, encouraging message (under 60 characters) for this scenario:

STUDENT CONTEXT:
- Current Level: $currentLevel
- Study Streak: $studyStreak days
- Daily Progress: $dailyProgress/$dailyGoal pomodoros
- Productivity Score: ${productivityScore.toInt()}%
- Study Pattern: $studyPattern
- Total Points: $totalPoints

SCENARIO: $type

''';

    switch (type) {
      case 'work_start':
        return basePrompt +
            '''
Generate an encouraging message to start a work session. Include an emoji and be motivational. Examples:
- "🚀 Let's crush this session!"
- "💪 Time to focus and excel!"
- "🎯 Ready to achieve greatness!"

Message:''';

      case 'work_complete':
        return basePrompt +
            '''
Generate a congratulatory message for completing a pomodoro. Be celebratory and encouraging. Examples:
- "🎉 Amazing work! Keep it up!"
- "⭐ You're on fire today!"
- "🔥 Another victory! Well done!"

Message:''';

      case 'break_end':
        return basePrompt +
            '''
Generate a message to return from break. Be energizing and motivating. Examples:
- "⚡ Break time over! Let's go!"
- "🎯 Refreshed and ready!"
- "💫 Time to get back to it!"

Message:''';

      default:
        return basePrompt + 'Generate a motivational message.';
    }
  }

  String _parseMotivationalMessage(String aiResponse) {
    // Extract the message from AI response
    final lines = aiResponse.split('\n');
    for (final line in lines) {
      final trimmed = line.trim();
      if (trimmed.isNotEmpty &&
          !trimmed.toLowerCase().contains('message:') &&
          trimmed.length < 60) {
        return trimmed;
      }
    }
    return '';
  }

  // AI Chat Methods
  Future<void> _sendAIMessage(String message) async {
    if (!_geminiInitialized || message.trim().isEmpty) return;

    // Add user message to chat history
    setState(() {
      aiChatHistory.add({
        'sender': 'user',
        'message': message.trim(),
        'timestamp': DateTime.now().toIso8601String(),
      });
    });

    try {
      final prompt = _buildChatPrompt(message);
      final response = await _geminiModel.generateContent([
        Content.text(prompt),
      ]);

      if (response.text != null) {
        setState(() {
          aiChatHistory.add({
            'sender': 'ai',
            'message': response.text!.trim(),
            'timestamp': DateTime.now().toIso8601String(),
          });
        });
      }
    } catch (e) {
      setState(() {
        aiChatHistory.add({
          'sender': 'ai',
          'message': 'Sorry, I encountered an error. Please try again.',
          'timestamp': DateTime.now().toIso8601String(),
        });
      });
    }

    chatController.clear();
  }

  String _buildChatPrompt(String userMessage) {
    return '''
You are Mentora, an AI Study Companion robot. You help students optimize their study environment and productivity.

CURRENT STUDENT CONTEXT:
- Name: Study Buddy
- Level: $currentLevel
- Study Streak: $studyStreak days
- Daily Progress: $dailyProgress/$dailyGoal pomodoros
- Productivity Score: ${productivityScore.toInt()}%
- Study Pattern: $studyPattern
- Total Points: $totalPoints

CURRENT ENVIRONMENT:
- Light: ${lightLevel.toInt()} lux
- Temperature: ${temperature.toInt()}°C
- Sound: $soundLevel dB
- Air Quality: ${airQuality.toInt()}%
- Motion: ${motionDetected ? 'Detected' : 'Stable'}
- Voice Activity: ${voiceActivity ? 'Active' : 'Quiet'}

RECENT ACHIEVEMENTS: ${achievements.where((a) => a.isUnlocked).map((a) => a.title).join(', ')}

USER MESSAGE: "$userMessage"

Respond as a friendly, encouraging AI study companion. Be helpful, motivational, and provide actionable advice. Keep responses conversational and under 200 characters. Use emojis appropriately.

Response:''';
  }

  void _toggleAIChat() {
    setState(() {
      isAIChatVisible = !isAIChatVisible;
    });
  }

  Widget _buildAIChatOverlay(ThemeData theme, bool isDark) {
    return Container(
      color: Colors.black.withOpacity(0.5),
      child: Center(
        child: Container(
          margin: const EdgeInsets.all(20),
          height: MediaQuery.of(context).size.height * 0.7,
          decoration: BoxDecoration(
            color: isDark ? const Color(0xFF1A1A2E) : Colors.white,
            borderRadius: BorderRadius.circular(20),
            boxShadow: [
              BoxShadow(
                color: Colors.black.withOpacity(0.3),
                blurRadius: 20,
                offset: const Offset(0, 10),
              ),
            ],
          ),
          child: Column(
            children: [
              // Chat Header
              Container(
                padding: const EdgeInsets.all(16),
                decoration: BoxDecoration(
                  gradient: LinearGradient(
                    colors: [const Color(0xFF6C63FF), const Color(0xFF5A52D5)],
                    begin: Alignment.topLeft,
                    end: Alignment.bottomRight,
                  ),
                  borderRadius: const BorderRadius.only(
                    topLeft: Radius.circular(20),
                    topRight: Radius.circular(20),
                  ),
                ),
                child: Row(
                  children: [
                    const Icon(Icons.psychology, color: Colors.white, size: 24),
                    const SizedBox(width: 12),
                    const Text(
                      'AI Study Companion',
                      style: TextStyle(
                        color: Colors.white,
                        fontSize: 18,
                        fontWeight: FontWeight.bold,
                      ),
                    ),
                    const Spacer(),
                    IconButton(
                      icon: const Icon(Icons.close, color: Colors.white),
                      onPressed: _toggleAIChat,
                    ),
                  ],
                ),
              ),

              // Chat Messages
              Expanded(
                child: ListView.builder(
                  padding: const EdgeInsets.all(16),
                  itemCount: aiChatHistory.length,
                  itemBuilder: (context, index) {
                    final message = aiChatHistory[index];
                    final isUser = message['sender'] == 'user';

                    return Padding(
                      padding: const EdgeInsets.symmetric(vertical: 4),
                      child: Row(
                        mainAxisAlignment: isUser
                            ? MainAxisAlignment.end
                            : MainAxisAlignment.start,
                        children: [
                          if (!isUser) ...[
                            Container(
                              width: 32,
                              height: 32,
                              decoration: BoxDecoration(
                                color: const Color(0xFF6C63FF).withOpacity(0.1),
                                borderRadius: BorderRadius.circular(16),
                              ),
                              child: const Icon(
                                Icons.psychology,
                                color: Color(0xFF6C63FF),
                                size: 16,
                              ),
                            ),
                            const SizedBox(width: 8),
                          ],
                          Flexible(
                            child: Container(
                              padding: const EdgeInsets.symmetric(
                                horizontal: 16,
                                vertical: 12,
                              ),
                              decoration: BoxDecoration(
                                color: isUser
                                    ? const Color(0xFF6C63FF)
                                    : Colors.grey.shade100,
                                borderRadius: BorderRadius.circular(20),
                              ),
                              child: Text(
                                message['message']!,
                                style: TextStyle(
                                  color: isUser ? Colors.white : Colors.black87,
                                  fontSize: 14,
                                ),
                              ),
                            ),
                          ),
                          if (isUser) ...[
                            const SizedBox(width: 8),
                            Container(
                              width: 32,
                              height: 32,
                              decoration: BoxDecoration(
                                color: const Color(0xFF6C63FF).withOpacity(0.1),
                                borderRadius: BorderRadius.circular(16),
                              ),
                              child: const Icon(
                                Icons.person,
                                color: Color(0xFF6C63FF),
                                size: 16,
                              ),
                            ),
                          ],
                        ],
                      ),
                    );
                  },
                ),
              ),

              // Chat Input
              Container(
                padding: const EdgeInsets.all(16),
                decoration: BoxDecoration(
                  color: Colors.grey.shade50,
                  borderRadius: const BorderRadius.only(
                    bottomLeft: Radius.circular(20),
                    bottomRight: Radius.circular(20),
                  ),
                ),
                child: Row(
                  children: [
                    Expanded(
                      child: TextField(
                        controller: chatController,
                        decoration: InputDecoration(
                          hintText: 'Ask me anything about your study...',
                          border: OutlineInputBorder(
                            borderRadius: BorderRadius.circular(25),
                            borderSide: BorderSide.none,
                          ),
                          filled: true,
                          fillColor: Colors.white,
                          contentPadding: const EdgeInsets.symmetric(
                            horizontal: 16,
                            vertical: 12,
                          ),
                        ),
                        onSubmitted: (value) {
                          if (value.trim().isNotEmpty) {
                            _sendAIMessage(value);
                          }
                        },
                      ),
                    ),
                    const SizedBox(width: 8),
                    FloatingActionButton(
                      mini: true,
                      onPressed: () {
                        if (chatController.text.trim().isNotEmpty) {
                          _sendAIMessage(chatController.text);
                        }
                      },
                      backgroundColor: const Color(0xFF6C63FF),
                      child: const Icon(Icons.send, color: Colors.white),
                    ),
                  ],
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  void _showNotification(String message, Color color) {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(message),
        backgroundColor: color,
        behavior: SnackBarBehavior.floating,
        duration: const Duration(seconds: 2),
      ),
    );
  }

  void _showAchievementDialog(Achievement achievement) {
    showDialog(
      context: context,
      builder: (BuildContext context) {
        return AlertDialog(
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(20),
          ),
          title: Row(
            children: [
              Text(achievement.icon, style: const TextStyle(fontSize: 32)),
              const SizedBox(width: 12),
              const Text('Achievement Unlocked!'),
            ],
          ),
          content: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              Text(
                achievement.title,
                style: const TextStyle(
                  fontSize: 20,
                  fontWeight: FontWeight.bold,
                ),
              ),
              const SizedBox(height: 8),
              Text(achievement.description),
              const SizedBox(height: 16),
              Container(
                padding: const EdgeInsets.all(12),
                decoration: BoxDecoration(
                  color: Colors.amber.withOpacity(0.1),
                  borderRadius: BorderRadius.circular(12),
                ),
                child: Text(
                  '+${achievement.points} points',
                  style: const TextStyle(
                    fontSize: 16,
                    fontWeight: FontWeight.bold,
                    color: Colors.amber,
                  ),
                ),
              ),
            ],
          ),
          actions: [
            FilledButton(
              onPressed: () => Navigator.of(context).pop(),
              child: const Text('Awesome!'),
            ),
          ],
        );
      },
    );
  }

  Future<void> _sendEmotion(String emotion) async {
    if (isLoading) return;

    setState(() {
      isLoading = true;
      statusMessage = 'Sending $emotion...';
    });

    try {
      final response = await http
          .post(
            Uri.parse('http://$esp32Ip/emotion'),
            headers: {'Content-Type': 'application/json'},
            body: json.encode({'emotion': emotion}),
          )
          .timeout(const Duration(seconds: 8));

      if (response.statusCode == 200) {
        final data = json.decode(response.body);
        setState(() {
          currentEmotion = data['emotion'] ?? emotion;
          baseEmotion = data['base_emotion'] ?? emotion;
          hasReaction = data['has_reaction'] ?? false;
          statusMessage = '$emotion sent successfully!';
          isConnected = true;
          connectionStatus = 'Connected';
        });

        _showSuccessSnackbar('$emotion command sent successfully!');

        Timer(const Duration(seconds: 2), () {
          if (mounted) {
            setState(() {
              statusMessage = 'Ready';
            });
          }
        });

        if (emotion.contains('REACTION') ||
            emotion == 'YES' ||
            emotion == 'NO') {
          Timer(const Duration(milliseconds: 500), () {
            _checkStatus();
          });
        }
      } else {
        throw Exception('HTTP ${response.statusCode}: ${response.body}');
      }
    } catch (e) {
      String errorMsg = e.toString().replaceAll('Exception: ', '');
      if (errorMsg.contains('TimeoutException')) {
        errorMsg = 'Connection timeout - check ESP32 IP';
      }

      setState(() {
        statusMessage = 'Error: $errorMsg';
        isConnected = false;
        connectionStatus = 'Connection Failed';
      });

      _showErrorSnackbar('Failed to send command: $errorMsg');

      Timer(const Duration(seconds: 3), () {
        if (mounted) {
          setState(() {
            statusMessage = 'Ready';
          });
        }
      });
    } finally {
      setState(() {
        isLoading = false;
      });
    }
  }

  void _showSuccessSnackbar(String message) {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Row(
          children: [
            const Icon(Icons.check_circle, color: Colors.white),
            const SizedBox(width: 8),
            Text(message),
          ],
        ),
        backgroundColor: Colors.green,
        behavior: SnackBarBehavior.floating,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(10)),
      ),
    );
  }

  void _showErrorSnackbar(String message) {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Row(
          children: [
            const Icon(Icons.error, color: Colors.white),
            const SizedBox(width: 8),
            Expanded(child: Text(message)),
          ],
        ),
        backgroundColor: Colors.red,
        behavior: SnackBarBehavior.floating,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(10)),
        action: SnackBarAction(
          label: 'Retry',
          textColor: Colors.white,
          onPressed: () => _checkStatus(),
        ),
      ),
    );
  }

  void _showIpDialog() {
    TextEditingController ipController = TextEditingController(text: esp32Ip);

    showDialog(
      context: context,
      builder: (BuildContext context) {
        return AlertDialog(
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(20),
          ),
          title: const Row(
            children: [
              Icon(Icons.settings_ethernet, color: Color(0xFF6C63FF)),
              SizedBox(width: 8),
              Text('ESP32 Configuration'),
            ],
          ),
          content: Column(
            mainAxisSize: MainAxisSize.min,
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              const Text('Enter your ESP32 IP address:'),
              const SizedBox(height: 12),
              TextField(
                controller: ipController,
                decoration: InputDecoration(
                  labelText: 'IP Address',
                  hintText: '192.168.1.100',
                  border: OutlineInputBorder(
                    borderRadius: BorderRadius.circular(12),
                  ),
                  prefixIcon: const Icon(Icons.wifi),
                ),
                keyboardType: TextInputType.number,
              ),
            ],
          ),
          actions: [
            TextButton(
              onPressed: () => Navigator.of(context).pop(),
              child: const Text('Cancel'),
            ),
            FilledButton(
              onPressed: () {
                setState(() {
                  esp32Ip = ipController.text.trim();
                });
                Navigator.of(context).pop();
                _checkStatus();
                _showSuccessSnackbar('IP address updated to $esp32Ip');
              },
              style: FilledButton.styleFrom(
                backgroundColor: const Color(0xFF6C63FF),
                shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.circular(12),
                ),
              ),
              child: const Text('Save'),
            ),
          ],
        );
      },
    );
  }

  Color _getScoreColor(double score) {
    if (score >= 80) return Colors.green;
    if (score >= 60) return Colors.orange;
    return Colors.red;
  }

  Color _getStatusColor(String status) {
    switch (status.toLowerCase()) {
      case 'good':
      case 'focused':
      case 'quiet':
      case 'stable':
      case 'normal':
        return Colors.green;
      case 'moderate':
      case 'distracted':
        return Colors.orange;
      case 'low':
      case 'high':
      case 'noisy':
      case 'moving':
      case 'very_distracted':
        return Colors.red;
      default:
        return Colors.grey;
    }
  }

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    final isDark = theme.brightness == Brightness.dark;

    return Scaffold(
      backgroundColor: isDark
          ? const Color(0xFF0F0F23)
          : const Color(0xFFF8F9FF),
      body: Stack(
        children: [
          // Main content
          CustomScrollView(
            slivers: [
              // Custom App Bar
              SliverAppBar(
                expandedHeight: 120,
                pinned: true,
                elevation: 0,
                backgroundColor: Colors.transparent,
                flexibleSpace: Container(
                  decoration: BoxDecoration(
                    gradient: LinearGradient(
                      colors: [
                        const Color(0xFF6C63FF),
                        const Color(0xFF5A52D5),
                      ],
                      begin: Alignment.topLeft,
                      end: Alignment.bottomRight,
                    ),
                  ),
                  child: FlexibleSpaceBar(
                    title: AnimatedBuilder(
                      animation: _pulseAnimation,
                      builder: (context, child) {
                        return Transform.scale(
                          scale: isConnected ? 1.0 : _pulseAnimation.value,
                          child: const Text(
                            'Mentora Robot',
                            style: TextStyle(
                              color: Colors.white,
                              fontWeight: FontWeight.bold,
                            ),
                          ),
                        );
                      },
                    ),
                    centerTitle: true,
                  ),
                ),
                actions: [
                  IconButton(
                    icon: Icon(
                      _geminiInitialized
                          ? Icons.psychology
                          : Icons.psychology_outlined,
                      color: _geminiInitialized ? Colors.white : Colors.white70,
                    ),
                    onPressed: _toggleAIChat,
                    tooltip: _geminiInitialized
                        ? 'AI Chat'
                        : 'AI Not Available',
                  ),
                  IconButton(
                    icon: const Icon(Icons.settings, color: Colors.white),
                    onPressed: _showIpDialog,
                  ),
                  IconButton(
                    icon: const Icon(Icons.refresh, color: Colors.white),
                    onPressed: _checkStatus,
                  ),
                ],
              ),

              // Main Content
              SliverPadding(
                padding: const EdgeInsets.all(16),
                sliver: SliverList(
                  delegate: SliverChildListDelegate([
                    // Connection Status Card
                    _buildConnectionCard(theme, isDark),
                    const SizedBox(height: 20),

                    // AI Analytics Dashboard
                    _buildAIAnalyticsCard(theme, isDark),
                    const SizedBox(height: 20),

                    // Pomodoro Timer
                    _buildPomodoroCard(theme, isDark),
                    const SizedBox(height: 20),

                    // Gamification Progress
                    _buildGamificationCard(theme, isDark),
                    const SizedBox(height: 20),

                    // Study Score Card
                    _buildStudyScoreCard(theme, isDark),
                    const SizedBox(height: 20),

                    // Sensor Grid
                    _buildSensorGrid(theme, isDark),
                    const SizedBox(height: 20),

                    // Insights Section
                    _buildInsightsSection(theme, isDark),
                    const SizedBox(height: 20),

                    // Emotion Controls
                    _buildEmotionControls(theme, isDark),
                    const SizedBox(height: 20),

                    // Response Controls
                    _buildResponseControls(theme, isDark),
                    const SizedBox(height: 100),
                  ]),
                ),
              ),
            ],
          ),

          // AI Chat Overlay
          if (isAIChatVisible) _buildAIChatOverlay(theme, isDark),
        ],
      ),
      floatingActionButton: isConnected
          ? FloatingActionButton(
              onPressed: () {
                _checkStatus();
                _fetchSensorData();
                _fetchInsights();
              },
              backgroundColor: const Color(0xFF6C63FF),
              child: const Icon(Icons.refresh, color: Colors.white),
            )
          : null,
    );
  }

  Widget _buildConnectionCard(ThemeData theme, bool isDark) {
    return Container(
      padding: const EdgeInsets.all(20),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: isConnected
              ? [Colors.green.shade400, Colors.green.shade600]
              : [Colors.red.shade400, Colors.red.shade600],
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
        ),
        borderRadius: BorderRadius.circular(20),
        boxShadow: [
          BoxShadow(
            color: (isConnected ? Colors.green : Colors.red).withOpacity(0.3),
            blurRadius: 15,
            offset: const Offset(0, 5),
          ),
        ],
      ),
      child: Row(
        children: [
          AnimatedBuilder(
            animation: _waveAnimation,
            builder: (context, child) {
              return Container(
                width: 60,
                height: 60,
                decoration: BoxDecoration(
                  shape: BoxShape.circle,
                  color: Colors.white.withOpacity(0.2),
                ),
                child: Icon(
                  isConnected ? Icons.wifi : Icons.wifi_off,
                  color: Colors.white,
                  size: 30,
                ),
              );
            },
          ),
          const SizedBox(width: 16),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  connectionStatus,
                  style: const TextStyle(
                    color: Colors.white,
                    fontSize: 18,
                    fontWeight: FontWeight.bold,
                  ),
                ),
                const SizedBox(height: 4),
                Text(
                  isConnected
                      ? 'Signal: $signalStrength dBm'
                      : 'Check connection',
                  style: TextStyle(
                    color: Colors.white.withOpacity(0.8),
                    fontSize: 14,
                  ),
                ),
                if (isConnected)
                  Text(
                    'Uptime: $uptime',
                    style: TextStyle(
                      color: Colors.white.withOpacity(0.8),
                      fontSize: 12,
                    ),
                  ),
              ],
            ),
          ),
          Text(
            currentEmotion,
            style: const TextStyle(
              color: Colors.white,
              fontSize: 16,
              fontWeight: FontWeight.bold,
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildAIAnalyticsCard(ThemeData theme, bool isDark) {
    return Container(
      padding: const EdgeInsets.all(24),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [
            const Color(0xFF6C63FF).withOpacity(0.1),
            const Color(0xFF5A52D5).withOpacity(0.1),
          ],
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
        ),
        color: isDark ? const Color(0xFF1A1A2E) : Colors.white,
        borderRadius: BorderRadius.circular(20),
        border: Border.all(
          color: const Color(0xFF6C63FF).withOpacity(0.3),
          width: 1,
        ),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.1),
            blurRadius: 20,
            offset: const Offset(0, 5),
          ),
        ],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              const Icon(Icons.psychology, color: Color(0xFF6C63FF), size: 28),
              const SizedBox(width: 12),
              const Text(
                'AI Analytics & Learning',
                style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
              ),
              const Spacer(),
              Container(
                padding: const EdgeInsets.symmetric(
                  horizontal: 12,
                  vertical: 6,
                ),
                decoration: BoxDecoration(
                  color: const Color(0xFF6C63FF).withOpacity(0.1),
                  borderRadius: BorderRadius.circular(20),
                ),
                child: Text(
                  currentLevel,
                  style: const TextStyle(
                    color: Color(0xFF6C63FF),
                    fontWeight: FontWeight.bold,
                    fontSize: 12,
                  ),
                ),
              ),
            ],
          ),
          const SizedBox(height: 20),
          Row(
            children: [
              Expanded(
                child: _buildAnalyticsMetric(
                  'Productivity Score',
                  '${productivityScore.toInt()}%',
                  _getScoreColor(productivityScore),
                  Icons.trending_up,
                ),
              ),
              const SizedBox(width: 16),
              Expanded(
                child: _buildAnalyticsMetric(
                  'Concentration Index',
                  '${concentrationIndex.toInt()}%',
                  _getScoreColor(concentrationIndex),
                  Icons.center_focus_strong,
                ),
              ),
            ],
          ),
          const SizedBox(height: 16),
          Row(
            children: [
              Expanded(
                child: _buildAnalyticsMetric(
                  'Study Pattern',
                  studyPattern.replaceAll('_', ' ').toUpperCase(),
                  Colors.blue,
                  Icons.schedule,
                ),
              ),
              const SizedBox(width: 16),
              Expanded(
                child: _buildAnalyticsMetric(
                  'Total Points',
                  totalPoints.toString(),
                  Colors.amber,
                  Icons.stars,
                ),
              ),
            ],
          ),
          if (personalizedRecommendations.isNotEmpty) ...[
            const SizedBox(height: 16),
            const Text(
              'AI Recommendations:',
              style: TextStyle(fontSize: 14, fontWeight: FontWeight.w600),
            ),
            const SizedBox(height: 8),
            ...personalizedRecommendations
                .take(3)
                .map(
                  (rec) => Padding(
                    padding: const EdgeInsets.symmetric(vertical: 2),
                    child: Text(
                      rec,
                      style: TextStyle(
                        fontSize: 12,
                        color: Colors.grey.shade600,
                      ),
                    ),
                  ),
                ),
          ],
        ],
      ),
    );
  }

  Widget _buildAnalyticsMetric(
    String label,
    String value,
    Color color,
    IconData icon,
  ) {
    return Container(
      padding: const EdgeInsets.all(12),
      decoration: BoxDecoration(
        color: color.withOpacity(0.1),
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: color.withOpacity(0.3)),
      ),
      child: Column(
        children: [
          Icon(icon, color: color, size: 20),
          const SizedBox(height: 8),
          Text(
            value,
            style: TextStyle(
              fontSize: 16,
              fontWeight: FontWeight.bold,
              color: color,
            ),
          ),
          Text(
            label,
            style: TextStyle(fontSize: 10, color: Colors.grey.shade600),
            textAlign: TextAlign.center,
          ),
        ],
      ),
    );
  }

  Widget _buildPomodoroCard(ThemeData theme, bool isDark) {
    return Container(
      padding: const EdgeInsets.all(24),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: pomodoroPhase == 'work'
              ? [Colors.red.withOpacity(0.1), Colors.orange.withOpacity(0.1)]
              : [Colors.green.withOpacity(0.1), Colors.teal.withOpacity(0.1)],
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
        ),
        color: isDark ? const Color(0xFF1A1A2E) : Colors.white,
        borderRadius: BorderRadius.circular(20),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.1),
            blurRadius: 20,
            offset: const Offset(0, 5),
          ),
        ],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Icon(
                pomodoroPhase == 'work' ? Icons.work : Icons.coffee,
                color: pomodoroPhase == 'work' ? Colors.red : Colors.green,
                size: 28,
              ),
              const SizedBox(width: 12),
              Text(
                'Pomodoro Timer',
                style: const TextStyle(
                  fontSize: 18,
                  fontWeight: FontWeight.bold,
                ),
              ),
              const Spacer(),
              Container(
                padding: const EdgeInsets.symmetric(
                  horizontal: 12,
                  vertical: 6,
                ),
                decoration: BoxDecoration(
                  color: pomodoroPhase == 'work'
                      ? Colors.red.withOpacity(0.1)
                      : Colors.green.withOpacity(0.1),
                  borderRadius: BorderRadius.circular(20),
                ),
                child: Text(
                  pomodoroPhase.toUpperCase().replaceAll('_', ' '),
                  style: TextStyle(
                    color: pomodoroPhase == 'work' ? Colors.red : Colors.green,
                    fontWeight: FontWeight.bold,
                    fontSize: 12,
                  ),
                ),
              ),
            ],
          ),
          const SizedBox(height: 20),
          Center(
            child: Text(
              _formatTime(remainingTime),
              style: TextStyle(
                fontSize: 48,
                fontWeight: FontWeight.bold,
                color: pomodoroPhase == 'work' ? Colors.red : Colors.green,
              ),
            ),
          ),
          const SizedBox(height: 20),
          Row(
            children: [
              Expanded(
                child: _buildPomodoroButton(
                  isPomodoroActive ? 'Pause' : 'Start',
                  isPomodoroActive ? Icons.pause : Icons.play_arrow,
                  isPomodoroActive ? Colors.orange : Colors.green,
                  isPomodoroActive ? _pausePomodoro : _startPomodoro,
                ),
              ),
              const SizedBox(width: 12),
              Expanded(
                child: _buildPomodoroButton(
                  'Stop',
                  Icons.stop,
                  Colors.red,
                  _stopPomodoro,
                ),
              ),
            ],
          ),
          const SizedBox(height: 16),
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceAround,
            children: [
              _buildPomodoroStat(
                'Completed',
                completedPomodoros.toString(),
                Icons.check_circle,
              ),
              _buildPomodoroStat(
                'Today',
                dailyProgress.toString(),
                Icons.today,
              ),
              _buildPomodoroStat('Goal', dailyGoal.toString(), Icons.flag),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildPomodoroButton(
    String text,
    IconData icon,
    Color color,
    VoidCallback onPressed,
  ) {
    return ElevatedButton.icon(
      onPressed: onPressed,
      icon: Icon(icon, color: Colors.white),
      label: Text(text, style: const TextStyle(color: Colors.white)),
      style: ElevatedButton.styleFrom(
        backgroundColor: color,
        padding: const EdgeInsets.symmetric(vertical: 12),
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
      ),
    );
  }

  Widget _buildPomodoroStat(String label, String value, IconData icon) {
    return Column(
      children: [
        Icon(icon, color: Colors.grey.shade600, size: 20),
        const SizedBox(height: 4),
        Text(
          value,
          style: const TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
        ),
        Text(
          label,
          style: TextStyle(fontSize: 12, color: Colors.grey.shade600),
        ),
      ],
    );
  }

  Widget _buildGamificationCard(ThemeData theme, bool isDark) {
    return Container(
      padding: const EdgeInsets.all(24),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [
            Colors.amber.withOpacity(0.1),
            Colors.orange.withOpacity(0.1),
          ],
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
        ),
        color: isDark ? const Color(0xFF1A1A2E) : Colors.white,
        borderRadius: BorderRadius.circular(20),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.1),
            blurRadius: 20,
            offset: const Offset(0, 5),
          ),
        ],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              const Icon(Icons.emoji_events, color: Colors.amber, size: 28),
              const SizedBox(width: 12),
              const Text(
                'Gamification & Progress',
                style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
              ),
              const Spacer(),
              Container(
                padding: const EdgeInsets.symmetric(
                  horizontal: 12,
                  vertical: 6,
                ),
                decoration: BoxDecoration(
                  color: Colors.amber.withOpacity(0.1),
                  borderRadius: BorderRadius.circular(20),
                ),
                child: Text(
                  'Level $currentLevel',
                  style: const TextStyle(
                    color: Colors.amber,
                    fontWeight: FontWeight.bold,
                    fontSize: 12,
                  ),
                ),
              ),
            ],
          ),
          const SizedBox(height: 20),
          Row(
            children: [
              Expanded(
                child: _buildGamificationMetric(
                  'Total Points',
                  totalPoints.toString(),
                  Colors.amber,
                  Icons.stars,
                ),
              ),
              const SizedBox(width: 16),
              Expanded(
                child: _buildGamificationMetric(
                  'Study Streak',
                  '${studyStreak} days',
                  Colors.green,
                  Icons.local_fire_department,
                ),
              ),
            ],
          ),
          const SizedBox(height: 16),
          Row(
            children: [
              Expanded(
                child: _buildGamificationMetric(
                  'Daily Progress',
                  '$dailyProgress/$dailyGoal',
                  Colors.blue,
                  Icons.today,
                ),
              ),
              const SizedBox(width: 16),
              Expanded(
                child: _buildGamificationMetric(
                  'Achievements',
                  '${achievements.where((a) => a.isUnlocked).length}/${achievements.length}',
                  Colors.purple,
                  Icons.emoji_events,
                ),
              ),
            ],
          ),
          const SizedBox(height: 16),
          const Text(
            'Recent Achievements:',
            style: TextStyle(fontSize: 14, fontWeight: FontWeight.w600),
          ),
          const SizedBox(height: 8),
          SizedBox(
            height: 40,
            child: ListView.builder(
              scrollDirection: Axis.horizontal,
              itemCount: achievements.where((a) => a.isUnlocked).length,
              itemBuilder: (context, index) {
                final unlockedAchievements = achievements
                    .where((a) => a.isUnlocked)
                    .toList();
                if (index >= unlockedAchievements.length)
                  return const SizedBox.shrink();
                final achievement = unlockedAchievements[index];
                return Container(
                  margin: const EdgeInsets.only(right: 8),
                  padding: const EdgeInsets.all(8),
                  decoration: BoxDecoration(
                    color: Colors.amber.withOpacity(0.1),
                    borderRadius: BorderRadius.circular(8),
                    border: Border.all(color: Colors.amber.withOpacity(0.3)),
                  ),
                  child: Row(
                    mainAxisSize: MainAxisSize.min,
                    children: [
                      Text(
                        achievement.icon,
                        style: const TextStyle(fontSize: 16),
                      ),
                      const SizedBox(width: 4),
                      Text(
                        achievement.title,
                        style: const TextStyle(
                          fontSize: 10,
                          fontWeight: FontWeight.bold,
                        ),
                      ),
                    ],
                  ),
                );
              },
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildGamificationMetric(
    String label,
    String value,
    Color color,
    IconData icon,
  ) {
    return Container(
      padding: const EdgeInsets.all(12),
      decoration: BoxDecoration(
        color: color.withOpacity(0.1),
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: color.withOpacity(0.3)),
      ),
      child: Column(
        children: [
          Icon(icon, color: color, size: 20),
          const SizedBox(height: 8),
          Text(
            value,
            style: TextStyle(
              fontSize: 16,
              fontWeight: FontWeight.bold,
              color: color,
            ),
          ),
          Text(
            label,
            style: TextStyle(fontSize: 10, color: Colors.grey.shade600),
            textAlign: TextAlign.center,
          ),
        ],
      ),
    );
  }

  Widget _buildStudyScoreCard(ThemeData theme, bool isDark) {
    return Container(
      padding: const EdgeInsets.all(24),
      decoration: BoxDecoration(
        color: isDark ? const Color(0xFF1A1A2E) : Colors.white,
        borderRadius: BorderRadius.circular(20),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.1),
            blurRadius: 20,
            offset: const Offset(0, 5),
          ),
        ],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              const Icon(Icons.psychology, color: Color(0xFF6C63FF), size: 28),
              const SizedBox(width: 12),
              const Text(
                'Study Environment Score',
                style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
              ),
            ],
          ),
          const SizedBox(height: 20),
          Stack(
            children: [
              Container(
                height: 8,
                decoration: BoxDecoration(
                  color: Colors.grey.shade300,
                  borderRadius: BorderRadius.circular(4),
                ),
              ),
              AnimatedContainer(
                duration: const Duration(milliseconds: 1000),
                height: 8,
                width:
                    (studyScore / 100) *
                    MediaQuery.of(context).size.width *
                    0.8,
                decoration: BoxDecoration(
                  gradient: LinearGradient(
                    colors: [
                      _getScoreColor(studyScore),
                      _getScoreColor(studyScore).withOpacity(0.8),
                    ],
                  ),
                  borderRadius: BorderRadius.circular(4),
                ),
              ),
            ],
          ),
          const SizedBox(height: 16),
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            children: [
              Text(
                '${studyScore.toInt()}/100',
                style: TextStyle(
                  fontSize: 24,
                  fontWeight: FontWeight.bold,
                  color: _getScoreColor(studyScore),
                ),
              ),
              Container(
                padding: const EdgeInsets.symmetric(
                  horizontal: 12,
                  vertical: 6,
                ),
                decoration: BoxDecoration(
                  color: _getScoreColor(studyScore).withOpacity(0.1),
                  borderRadius: BorderRadius.circular(20),
                ),
                child: Text(
                  studyScore >= 80
                      ? 'Excellent'
                      : studyScore >= 60
                      ? 'Good'
                      : 'Needs Improvement',
                  style: TextStyle(
                    color: _getScoreColor(studyScore),
                    fontWeight: FontWeight.bold,
                    fontSize: 12,
                  ),
                ),
              ),
            ],
          ),
          const SizedBox(height: 12),
          Text(
            recommendation,
            style: TextStyle(color: Colors.grey.shade600, fontSize: 14),
          ),
        ],
      ),
    );
  }

  Widget _buildSensorGrid(ThemeData theme, bool isDark) {
    return GridView.count(
      shrinkWrap: true,
      physics: const NeverScrollableScrollPhysics(),
      crossAxisCount: 2,
      mainAxisSpacing: 16,
      crossAxisSpacing: 16,
      childAspectRatio: 1.1,
      children: [
        _buildSensorCard(
          icon: Icons.lightbulb_outline,
          title: 'Light Level',
          value: '${lightLevel.toInt()} lux',
          subtitle: lightStatus,
          color: _getStatusColor(lightStatus),
          isDark: isDark,
        ),
        _buildSensorCard(
          icon: Icons.favorite,
          title: 'Heart Rate',
          value: heartRateValid ? '${heartRate.toInt()} bpm' : 'Invalid',
          subtitle: heartRateValid ? 'SpO2: ${spO2.toInt()}%' : 'Place finger',
          color: heartRateValid ? Colors.red : Colors.grey,
          isDark: isDark,
          showPulse: heartRateValid,
        ),
        _buildSensorCard(
          icon: Icons.volume_up,
          title: 'Sound Level',
          value: soundLevel.toString(),
          subtitle: environmentStatus,
          color: _getStatusColor(environmentStatus),
          isDark: isDark,
        ),
        _buildSensorCard(
          icon: Icons.touch_app,
          title: 'Interactions',
          value: interactionCount.toString(),
          subtitle: 'Touch count',
          color: interactionCount > 0 ? Colors.blue : Colors.grey,
          isDark: isDark,
        ),
      ],
    );
  }

  Widget _buildSensorCard({
    required IconData icon,
    required String title,
    required String value,
    required String subtitle,
    required Color color,
    required bool isDark,
    bool showPulse = false,
  }) {
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: isDark ? const Color(0xFF1A1A2E) : Colors.white,
        borderRadius: BorderRadius.circular(16),
        border: Border.all(color: color.withOpacity(0.3), width: 1),
        boxShadow: [
          BoxShadow(
            color: color.withOpacity(0.1),
            blurRadius: 10,
            offset: const Offset(0, 3),
          ),
        ],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              AnimatedBuilder(
                animation: showPulse ? _breathingAnimation : _pulseController,
                builder: (context, child) {
                  return Transform.scale(
                    scale: showPulse ? _breathingAnimation.value : 1.0,
                    child: Icon(icon, color: color, size: 24),
                  );
                },
              ),
              const SizedBox(width: 8),
              Expanded(
                child: Text(
                  title,
                  style: TextStyle(
                    fontSize: 12,
                    color: Colors.grey.shade600,
                    fontWeight: FontWeight.w500,
                  ),
                ),
              ),
            ],
          ),
          const SizedBox(height: 12),
          Text(
            value,
            style: TextStyle(
              fontSize: 18,
              fontWeight: FontWeight.bold,
              color: color,
            ),
          ),
          const SizedBox(height: 4),
          Text(
            subtitle,
            style: TextStyle(fontSize: 11, color: Colors.grey.shade500),
          ),
        ],
      ),
    );
  }

  Widget _buildInsightsSection(ThemeData theme, bool isDark) {
    return Container(
      padding: const EdgeInsets.all(20),
      decoration: BoxDecoration(
        color: isDark ? const Color(0xFF1A1A2E) : Colors.white,
        borderRadius: BorderRadius.circular(16),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.1),
            blurRadius: 10,
            offset: const Offset(0, 3),
          ),
        ],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          const Row(
            children: [
              Icon(Icons.face, color: Color(0xFF6C63FF), size: 24),
              SizedBox(width: 8),
              Text(
                'Emotion Controls',
                style: TextStyle(fontSize: 16, fontWeight: FontWeight.bold),
              ),
            ],
          ),
          const SizedBox(height: 16),
          GridView.count(
            shrinkWrap: true,
            physics: const NeverScrollableScrollPhysics(),
            crossAxisCount: 2,
            mainAxisSpacing: 12,
            crossAxisSpacing: 12,
            childAspectRatio: 2.5,
            children: [
              _buildEmotionButton('DEFAULT', '😐', Colors.blue),
              _buildEmotionButton('HAPPY', '😊', Colors.green),
              _buildEmotionButton('ANGRY', '😠', Colors.red),
              _buildEmotionButton('TIRED', '😴', Colors.orange),
            ],
          ),
          const SizedBox(height: 16),
          const Text(
            'Reaction Emotions',
            style: TextStyle(fontSize: 14, fontWeight: FontWeight.w600),
          ),
          const SizedBox(height: 8),
          GridView.count(
            shrinkWrap: true,
            physics: const NeverScrollableScrollPhysics(),
            crossAxisCount: 2,
            mainAxisSpacing: 12,
            crossAxisSpacing: 12,
            childAspectRatio: 2.2,
            children: [
              _buildReactionButton('HAPPY_REACTION', '🎉', Colors.green),
              _buildReactionButton('ANGRY_REACTION', '💢', Colors.red),
              _buildReactionButton('TIRED_REACTION', '😪', Colors.orange),
              _buildReactionButton('DEFAULT_REACTION', '🔄', Colors.blue),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildResponseControls(ThemeData theme, bool isDark) {
    return Container(
      padding: const EdgeInsets.all(20),
      decoration: BoxDecoration(
        color: isDark ? const Color(0xFF1A1A2E) : Colors.white,
        borderRadius: BorderRadius.circular(16),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.1),
            blurRadius: 10,
            offset: const Offset(0, 3),
          ),
        ],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          const Row(
            children: [
              Icon(Icons.gesture, color: Color(0xFF6C63FF), size: 24),
              SizedBox(width: 8),
              Text(
                'Physical Responses',
                style: TextStyle(fontSize: 16, fontWeight: FontWeight.bold),
              ),
            ],
          ),
          const SizedBox(height: 16),
          Row(
            children: [
              Expanded(
                child: _buildResponseButton(
                  'YES',
                  '✅',
                  'Nod Up/Down',
                  Colors.teal,
                  isDark,
                ),
              ),
              const SizedBox(width: 12),
              Expanded(
                child: _buildResponseButton(
                  'NO',
                  '❌',
                  'Shake Left/Right',
                  Colors.deepOrange,
                  isDark,
                ),
              ),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildEmotionButton(String emotion, String emoji, Color color) {
    bool isCurrentEmotion = currentEmotion == emotion;
    bool isDisabled = isLoading;

    return Container(
      decoration: BoxDecoration(
        gradient: isCurrentEmotion
            ? LinearGradient(
                colors: [color.withOpacity(0.8), color],
                begin: Alignment.topLeft,
                end: Alignment.bottomRight,
              )
            : null,
        color: isCurrentEmotion ? null : Colors.grey.shade100,
        borderRadius: BorderRadius.circular(12),
        border: Border.all(
          color: isCurrentEmotion ? color : Colors.grey.shade300,
          width: isCurrentEmotion ? 2 : 1,
        ),
        boxShadow: isCurrentEmotion
            ? [
                BoxShadow(
                  color: color.withOpacity(0.3),
                  blurRadius: 8,
                  offset: const Offset(0, 3),
                ),
              ]
            : null,
      ),
      child: Material(
        color: Colors.transparent,
        child: InkWell(
          onTap: isDisabled ? null : () => _sendEmotion(emotion),
          borderRadius: BorderRadius.circular(12),
          child: Container(
            padding: const EdgeInsets.all(12),
            child: isLoading && currentEmotion == emotion
                ? const Center(
                    child: SizedBox(
                      width: 20,
                      height: 20,
                      child: CircularProgressIndicator(strokeWidth: 2),
                    ),
                  )
                : Row(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: [
                      Text(emoji, style: const TextStyle(fontSize: 20)),
                      const SizedBox(width: 8),
                      Text(
                        emotion,
                        style: TextStyle(
                          fontSize: 12,
                          fontWeight: isCurrentEmotion
                              ? FontWeight.bold
                              : FontWeight.normal,
                          color: isCurrentEmotion
                              ? Colors.white
                              : Colors.black87,
                        ),
                      ),
                    ],
                  ),
          ),
        ),
      ),
    );
  }

  Widget _buildReactionButton(String emotion, String emoji, Color color) {
    bool isCurrentEmotion = currentEmotion == emotion;
    bool isDisabled = isLoading;
    String displayName = emotion.replaceAll('_REACTION', '');

    return Container(
      decoration: BoxDecoration(
        gradient: isCurrentEmotion
            ? LinearGradient(
                colors: [color.withOpacity(0.8), Colors.amber.withOpacity(0.8)],
                begin: Alignment.topLeft,
                end: Alignment.bottomRight,
              )
            : null,
        color: isCurrentEmotion ? null : Colors.grey.shade50,
        borderRadius: BorderRadius.circular(12),
        border: Border.all(
          color: isCurrentEmotion ? Colors.amber : Colors.grey.shade300,
          width: isCurrentEmotion ? 2 : 1,
        ),
        boxShadow: isCurrentEmotion
            ? [
                BoxShadow(
                  color: Colors.amber.withOpacity(0.3),
                  blurRadius: 8,
                  offset: const Offset(0, 3),
                ),
              ]
            : null,
      ),
      child: Material(
        color: Colors.transparent,
        child: InkWell(
          onTap: isDisabled ? null : () => _sendEmotion(emotion),
          borderRadius: BorderRadius.circular(12),
          child: Container(
            padding: const EdgeInsets.all(8),
            child: isLoading && currentEmotion == emotion
                ? const Center(
                    child: SizedBox(
                      width: 20,
                      height: 20,
                      child: CircularProgressIndicator(strokeWidth: 2),
                    ),
                  )
                : Column(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: [
                      Row(
                        mainAxisAlignment: MainAxisAlignment.center,
                        children: [
                          Text(emoji, style: const TextStyle(fontSize: 16)),
                          const SizedBox(width: 4),
                          const Text('✨', style: TextStyle(fontSize: 12)),
                        ],
                      ),
                      const SizedBox(height: 4),
                      Text(
                        displayName,
                        style: TextStyle(
                          fontSize: 10,
                          fontWeight: isCurrentEmotion
                              ? FontWeight.bold
                              : FontWeight.normal,
                          color: isCurrentEmotion
                              ? Colors.white
                              : Colors.black87,
                        ),
                      ),
                      Text(
                        'REACTION',
                        style: TextStyle(
                          fontSize: 8,
                          color: isCurrentEmotion
                              ? Colors.white70
                              : Colors.amber,
                          fontWeight: FontWeight.bold,
                        ),
                      ),
                    ],
                  ),
          ),
        ),
      ),
    );
  }

  Widget _buildResponseButton(
    String emotion,
    String emoji,
    String action,
    Color color,
    bool isDark,
  ) {
    bool isCurrentEmotion = currentEmotion == emotion;
    bool isDisabled = isLoading;

    return Container(
      height: 80,
      decoration: BoxDecoration(
        gradient: isCurrentEmotion
            ? LinearGradient(
                colors: [color.withOpacity(0.8), color],
                begin: Alignment.topLeft,
                end: Alignment.bottomRight,
              )
            : LinearGradient(
                colors: [
                  isDark ? const Color(0xFF2A2A3E) : Colors.grey.shade50,
                  isDark ? const Color(0xFF2A2A3E) : Colors.grey.shade100,
                ],
              ),
        borderRadius: BorderRadius.circular(16),
        border: Border.all(
          color: isCurrentEmotion ? color : Colors.grey.shade300,
          width: isCurrentEmotion ? 2 : 1,
        ),
        boxShadow: isCurrentEmotion
            ? [
                BoxShadow(
                  color: color.withOpacity(0.4),
                  blurRadius: 12,
                  offset: const Offset(0, 4),
                ),
              ]
            : [
                BoxShadow(
                  color: Colors.black.withOpacity(0.1),
                  blurRadius: 6,
                  offset: const Offset(0, 2),
                ),
              ],
      ),
      child: Material(
        color: Colors.transparent,
        child: InkWell(
          onTap: isDisabled ? null : () => _sendEmotion(emotion),
          borderRadius: BorderRadius.circular(16),
          child: Container(
            padding: const EdgeInsets.all(12),
            child: isLoading && currentEmotion == emotion
                ? const Center(
                    child: SizedBox(
                      width: 24,
                      height: 24,
                      child: CircularProgressIndicator(strokeWidth: 3),
                    ),
                  )
                : Column(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: [
                      Text(emoji, style: const TextStyle(fontSize: 24)),
                      const SizedBox(height: 4),
                      Text(
                        emotion,
                        style: TextStyle(
                          fontSize: 14,
                          fontWeight: FontWeight.bold,
                          color: isCurrentEmotion ? Colors.white : null,
                        ),
                      ),
                      Text(
                        action,
                        style: TextStyle(
                          fontSize: 10,
                          color: isCurrentEmotion
                              ? Colors.white70
                              : Colors.grey.shade600,
                        ),
                      ),
                    ],
                  ),
          ),
        ),
      ),
    );
  }

  Widget _buildInsightsSection(ThemeData theme, bool isDark) {
    return Container(
      padding: const EdgeInsets.all(20),
      decoration: BoxDecoration(
        color: isDark ? const Color(0xFF1A1A2E) : Colors.white,
        borderRadius: BorderRadius.circular(16),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.1),
            blurRadius: 10,
            offset: const Offset(0, 3),
          ),
        ],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          const Row(
            children: [
              Icon(Icons.insights, color: Color(0xFF6C63FF), size: 24),
              SizedBox(width: 8),
              Text(
                'Study Insights',
                style: TextStyle(fontSize: 16, fontWeight: FontWeight.bold),
              ),
            ],
          ),
          const SizedBox(height: 16),
          _buildInsightRow('Focus Level', focusLevel),
          _buildInsightRow('Stress Level', stressLevel),
          _buildInsightRow('Position', positionStatus),
          _buildInsightRow('Environment', environmentStatus),
        ],
      ),
    );
  }

  Widget _buildInsightRow(String label, String value) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 4),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: [
          Text(
            label,
            style: TextStyle(color: Colors.grey.shade600, fontSize: 14),
          ),
          Container(
            padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
            decoration: BoxDecoration(
              color: _getStatusColor(value).withOpacity(0.1),
              borderRadius: BorderRadius.circular(12),
            ),
            child: Text(
              value,
              style: TextStyle(
                color: _getStatusColor(value),
                fontSize: 12,
                fontWeight: FontWeight.bold,
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildEmotionControls(ThemeData theme, bool isDark) {
    return Container(
      padding: const EdgeInsets.all(20),
      decoration: BoxDecoration(
        color: isDark ? const Color(0xFF1A1A2E) : Colors.white,
        borderRadius: BorderRadius.circular(16),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.1),
            blurRadius: 10,
            offset: const Offset(0, 3),
          ),
        ],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          const Row(
            children: [
              Icon(Icons.face, color: Color(0xFF6C63FF), size: 24),
              SizedBox(width: 8),
              Text(
                'Emotion Controls',
                style: TextStyle(fontSize: 16, fontWeight: FontWeight.bold),
              ),
            ],
          ),
          const SizedBox(height: 16),
          GridView.count(
            shrinkWrap: true,
            physics: const NeverScrollableScrollPhysics(),
            crossAxisCount: 2,
            mainAxisSpacing: 12,
            crossAxisSpacing: 12,
            childAspectRatio: 2.5,
            children: [
              _buildEmotionButton('DEFAULT', '😐', Colors.blue),
              _buildEmotionButton('HAPPY', '😊', Colors.green),
              _buildEmotionButton('ANGRY', '😠', Colors.red),
              _buildEmotionButton('TIRED', '😴', Colors.orange),
            ],
          ),
          const SizedBox(height: 16),
          const Text(
            'Reaction Emotions',
            style: TextStyle(fontSize: 14, fontWeight: FontWeight.w600),
          ),
          const SizedBox(height: 8),
          GridView.count(
            shrinkWrap: true,
            physics: const NeverScrollableScrollPhysics(),
            crossAxisCount: 2,
            mainAxisSpacing: 12,
            crossAxisSpacing: 12,
            childAspectRatio: 2.2,
            children: [
              _buildReactionButton('HAPPY_REACTION', '🎉', Colors.green),
              _buildReactionButton('ANGRY_REACTION', '💢', Colors.red),
              _buildReactionButton('TIRED_REACTION', '😪', Colors.orange),
              _buildReactionButton('DEFAULT_REACTION', '🔄', Colors.blue),
            ],
          ),
        ],
      ),
    );
  }
}
